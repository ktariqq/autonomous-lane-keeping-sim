"""Node graph wiring.

    world_node                          -> /world/traffic_light_state
    camera_node        (/vehicle/state) -> /camera/image_raw
    lane_detector_node (/camera/image_raw) -> /perception/lane_error, /perception/lane_detected
    traffic_light_detector_node (/camera/image_raw) -> /perception/traffic_light_state
    planner_node (/perception/traffic_light_state, /vehicle/state) -> /planner/target_speed, /planner/mode
    pid_controller_node (/perception/lane_error) -> /control/steering_cmd
    vehicle_node (/control/steering_cmd, /planner/target_speed) -> /vehicle/state

Each node is a thin adapter over the underlying component (unchanged
math/thresholds/gains), so the pipeline can be read/graphed the same way
a ROS 2 graph would be, without depending on rclpy or a simulator backend."""

import numpy as np
import cv2
import pygame


class WorldNode:
    TOPIC_LIGHT_STATE = "/world/traffic_light_state"

    def __init__(self, bus, traffic_light_system):
        self.bus = bus
        self.system = traffic_light_system
        self.bus.publish(self.TOPIC_LIGHT_STATE, self.system.state)

    def force(self, state):
        self.system.force(state)

    def spin_once(self, dt):
        self.system.update(dt)
        self.bus.publish(self.TOPIC_LIGHT_STATE, self.system.state)


class CameraNode:
    TOPIC_IMAGE = "/camera/image_raw"

    def __init__(self, bus, camera, world_surface_provider):
        self.bus = bus
        self.camera = camera
        self._get_world_surface = world_surface_provider

    def spin_once(self, vehicle):
        world_surface = self._get_world_surface()
        world_rgb = np.ascontiguousarray(
            np.transpose(pygame.surfarray.array3d(world_surface), (1, 0, 2))
        )
        world_bgr = cv2.cvtColor(world_rgb, cv2.COLOR_RGB2BGR)
        frame_rgb = self.camera.capture(world_bgr, vehicle)
        self.bus.publish(self.TOPIC_IMAGE, frame_rgb)
        return frame_rgb


class LaneDetectorNode:
    TOPIC_LANE_ERROR = "/perception/lane_error"
    TOPIC_LANE_DETECTED = "/perception/lane_detected"

    def __init__(self, bus, detector):
        self.bus = bus
        self.detector = detector

    def spin_once(self):
        frame = self.bus.get(CameraNode.TOPIC_IMAGE)
        result = self.detector.process(frame)
        self.bus.publish(self.TOPIC_LANE_ERROR, result["lane_error"])
        self.bus.publish(self.TOPIC_LANE_DETECTED, result["detected"])
        return result


class TrafficLightDetectorNode:
    TOPIC_LIGHT_DETECTED = "/perception/traffic_light_state"

    def __init__(self, bus, detector):
        self.bus = bus
        self.detector = detector

    def spin_once(self):
        frame = self.bus.get(CameraNode.TOPIC_IMAGE)
        state, counts = self.detector.process(frame)
        self.bus.publish(self.TOPIC_LIGHT_DETECTED, state)
        return state, counts


class PlannerNode:
    TOPIC_TARGET_SPEED = "/planner/target_speed"
    TOPIC_MODE = "/planner/mode"

    def __init__(self, bus, planner):
        self.bus = bus
        self.planner = planner

    def spin_once(self, base_target_speed, nearest_dist, vehicle_v):
        light_state = self.bus.get(TrafficLightDetectorNode.TOPIC_LIGHT_DETECTED)
        target_speed, mode = self.planner.plan(base_target_speed, nearest_dist, vehicle_v, light_state)
        self.bus.publish(self.TOPIC_TARGET_SPEED, target_speed)
        self.bus.publish(self.TOPIC_MODE, mode)
        return target_speed, mode


class PidControllerNode:
    TOPIC_STEER_CMD = "/control/steering_cmd"

    def __init__(self, bus, pid):
        self.bus = bus
        self.pid = pid

    def spin_once(self, dt):
        lane_error = self.bus.get(LaneDetectorNode.TOPIC_LANE_ERROR, 0.0)
        steer_cmd = self.pid.update(lane_error, dt)
        self.bus.publish(self.TOPIC_STEER_CMD, steer_cmd)
        return steer_cmd


class VehicleNode:
    TOPIC_STATE = "/vehicle/state"

    def __init__(self, bus, vehicle):
        self.bus = bus
        self.vehicle = vehicle
        self._publish_state()

    def _publish_state(self):
        v = self.vehicle
        self.bus.publish(self.TOPIC_STATE, {
            "x": v.x, "y": v.y, "theta": v.theta, "v": v.v, "delta": v.delta,
        })

    def spin_once(self, dt):
        steer_cmd = self.bus.get(PidControllerNode.TOPIC_STEER_CMD, 0.0)
        target_speed = self.bus.get(PlannerNode.TOPIC_TARGET_SPEED, 0.0)
        self.vehicle.update(dt, target_speed, steer_cmd)
        self._publish_state()