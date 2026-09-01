"""Entry point: assembles the node graph and runs the pygame main loop.

Run:
    python -m src.main

Controls:
    ESC        quit
    R          reset vehicle
    1 / 2 / 3  force traffic light to RED / YELLOW / GREEN
    UP / DOWN  increase / decrease cruise speed
    Q / A      increase / decrease Kp
    W / S      increase / decrease Kd
    E / D      increase / decrease Ki
"""

import math
import sys

import numpy as np
import pygame

from src import config as cfg
from src.world.track import Track
from src.world.scene import build_static_world
from src.world.traffic_light import TrafficLightSystem
from src.sensing.camera import Camera
from src.perception.lane_detector import LaneDetector
from src.perception.traffic_light_detector import TrafficLightDetector
from src.planning.planner import Planner
from src.control.pid import PIDController
from src.dynamics.vehicle import Vehicle
from src.core.bus import Bus
from src.core.nodes import (
    WorldNode, CameraNode, LaneDetectorNode, TrafficLightDetectorNode,
    PlannerNode, PidControllerNode, VehicleNode,
)
from src.rendering.draw import draw_traffic_lights, draw_vehicle
from src.rendering.hud import make_hud_fonts, draw_text


def main():
    pygame.init()
    screen = pygame.display.set_mode((cfg.SCREEN_W, cfg.SCREEN_H))
    pygame.display.set_caption("Autonomous Lane-Keeping Simulator")
    clock = pygame.time.Clock()
    font, font_small, font_big = make_hud_fonts()

    track = Track()
    static_world_surface = build_static_world(track)
    world_surface = pygame.Surface((cfg.WORLD_W, cfg.WORLD_H))

    bus = Bus()
    vehicle = Vehicle()
    camera = Camera()
    lane_detector = LaneDetector(cfg.CAM_W, cfg.CAM_H)
    light_detector = TrafficLightDetector(cfg.CAM_W, cfg.CAM_H)
    pid = PIDController(
        kp=cfg.PID_KP, ki=cfg.PID_KI, kd=cfg.PID_KD,
        out_limit=Vehicle.MAX_STEER, i_limit=cfg.PID_I_LIMIT,
    )
    planner = Planner(cfg.STOP_DISTANCE)

    world_node = WorldNode(bus, TrafficLightSystem())
    camera_node = CameraNode(bus, camera, lambda: world_surface)
    lane_detector_node = LaneDetectorNode(bus, lane_detector)
    traffic_light_detector_node = TrafficLightDetectorNode(bus, light_detector)
    planner_node = PlannerNode(bus, planner)
    pid_controller_node = PidControllerNode(bus, pid)
    vehicle_node = VehicleNode(bus, vehicle)

    base_target_speed = cfg.BASE_TARGET_SPEED
    running = True

    while running:
        dt = min(clock.tick(cfg.FPS) / 1000.0, 0.05)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_r:
                    vehicle.reset()
                    pid.reset()
                elif event.key == pygame.K_1:
                    world_node.force("red")
                elif event.key == pygame.K_2:
                    world_node.force("yellow")
                elif event.key == pygame.K_3:
                    world_node.force("green")

        keys = pygame.key.get_pressed()
        if keys[pygame.K_UP]:
            base_target_speed = min(150.0, base_target_speed + 40 * dt)
        if keys[pygame.K_DOWN]:
            base_target_speed = max(0.0, base_target_speed - 40 * dt)
        if keys[pygame.K_q]:
            pid.kp = min(10.0, pid.kp + 1.0 * dt)
        if keys[pygame.K_a]:
            pid.kp = max(0.0, pid.kp - 1.0 * dt)
        if keys[pygame.K_w]:
            pid.kd = min(6.0, pid.kd + 0.5 * dt)
        if keys[pygame.K_s]:
            pid.kd = max(0.0, pid.kd - 0.5 * dt)
        if keys[pygame.K_e]:
            pid.ki = min(0.2, pid.ki + 0.01 * dt)
        if keys[pygame.K_d]:
            pid.ki = max(0.0, pid.ki - 0.01 * dt)

        # --- node graph spin, one tick, topological order ------------------
        world_node.spin_once(dt)

        world_surface.blit(static_world_surface, (0, 0))
        draw_traffic_lights(world_surface, track, world_node.system.state)

        cam_frame_rgb = camera_node.spin_once(vehicle)
        lane_result = lane_detector_node.spin_once()
        light_state_detected, _ = traffic_light_detector_node.spin_once()

        nearest_dist = track.nearest_intersection_distance(vehicle.x, vehicle.y)
        target_speed, mode = planner_node.spin_once(base_target_speed, nearest_dist, vehicle.v)
        steer_cmd = pid_controller_node.spin_once(dt)
        vehicle_node.spin_once(dt)

        _render(
            screen, font, font_small, font_big, clock,
            track, world_surface, vehicle, cam_frame_rgb, lane_detector, lane_result,
            world_node, light_state_detected, mode, pid, base_target_speed,
            nearest_dist, target_speed, steer_cmd,
        )

    pygame.quit()
    sys.exit()


def _render(screen, font, font_small, font_big, clock, track, world_surface, vehicle,
            cam_frame_rgb, lane_detector, lane_result, world_node, light_state_detected,
            mode, pid, base_target_speed, nearest_dist, target_speed, steer_cmd):
    screen.fill((6, 5, 18))

    view_x = max(0, min(cfg.WORLD_W - cfg.MAIN_W, int(vehicle.x - cfg.MAIN_W / 2)))
    view_y = max(0, min(cfg.WORLD_H - cfg.MAIN_H, int(vehicle.y - cfg.MAIN_H / 2)))
    main_view = world_surface.subsurface((view_x, view_y, cfg.MAIN_W, cfg.MAIN_H))
    screen.blit(main_view, (0, 0))
    draw_vehicle(screen, vehicle, -view_x, -view_y)
    pygame.draw.rect(screen, cfg.COL_PANEL_BORDER, (0, 0, cfg.MAIN_W, cfg.MAIN_H), 2)

    cam_arr = np.ascontiguousarray(np.transpose(cam_frame_rgb, (1, 0, 2)))
    cam_surf = pygame.surfarray.make_surface(cam_arr)
    cam_disp_w = int(cfg.CAM_W * cfg.CAM_DISPLAY_SCALE)
    cam_disp_h = int(cfg.CAM_H * cfg.CAM_DISPLAY_SCALE)
    cam_surf = pygame.transform.smoothscale(cam_surf, (cam_disp_w, cam_disp_h))
    cam_x, cam_y = cfg.MAIN_W + 18, 18
    screen.blit(cam_surf, (cam_x, cam_y))
    pygame.draw.rect(screen, cfg.COL_PANEL_BORDER, (cam_x, cam_y, cam_disp_w, cam_disp_h), 2)
    draw_text(screen, font_small, "FRONT CAMERA", (cam_x, cam_y - 20), cfg.COL_TEXT_DIM)

    band_y0 = cam_y + int(lane_detector.band_top * cfg.CAM_DISPLAY_SCALE)
    band_y1 = cam_y + int(lane_detector.band_bottom * cfg.CAM_DISPLAY_SCALE)
    pygame.draw.rect(screen, (255, 255, 0), (cam_x, band_y0, cam_disp_w, band_y1 - band_y0), 1)
    det_x = cam_x + int(lane_result["center_x"] * cfg.CAM_DISPLAY_SCALE)
    pygame.draw.line(screen, (255, 0, 255), (det_x, band_y0), (det_x, band_y1), 2)
    mid_x = cam_x + cam_disp_w // 2
    pygame.draw.line(screen, (0, 255, 0), (mid_x, cam_y), (mid_x, cam_y + cam_disp_h), 1)

    panel_y = cam_y + cam_disp_h + 16
    panel_rect = (cfg.MAIN_W, panel_y - 8, cfg.SIDE_W, cfg.SCREEN_H - panel_y - cfg.HUD_H + 8)
    pygame.draw.rect(screen, cfg.COL_PANEL_BG, panel_rect)
    pygame.draw.rect(screen, cfg.COL_PANEL_BORDER, panel_rect, 2)

    rows = [
        ("MODE", mode, cfg.GREEN if mode == "FOLLOWING" else (cfg.YELLOW if mode == "BRAKING" else cfg.RED)),
        ("TRUE LIGHT", world_node.system.state.upper(),
         {"red": cfg.RED, "yellow": cfg.YELLOW, "green": cfg.GREEN}[world_node.system.state]),
        ("DETECTED LIGHT", light_state_detected.upper(), cfg.COL_TEXT),
        ("LANE DETECTED", "YES" if lane_result["detected"] else "NO (holding)", cfg.COL_TEXT),
        ("LANE ERROR", f"{lane_result['lane_error']:+.1f} px", cfg.COL_TEXT),
        ("", "", cfg.COL_TEXT),
        ("PID Kp", f"{pid.kp:.2f}", cfg.COL_TEXT),
        ("PID Ki", f"{pid.ki:.3f}", cfg.COL_TEXT),
        ("PID Kd", f"{pid.kd:.2f}", cfg.COL_TEXT),
        ("P / I / D", f"{pid.p_term:+.1f} / {pid.i_term:+.1f} / {pid.d_term:+.1f}", cfg.COL_TEXT_DIM),
        ("", "", cfg.COL_TEXT),
        ("CRUISE TARGET", f"{base_target_speed:.0f}", cfg.COL_TEXT),
        ("DIST TO LIGHT", f"{nearest_dist:.0f}", cfg.COL_TEXT),
    ]
    ly = panel_y
    for label, value, color in rows:
        if label:
            draw_text(screen, font_small, label, (cfg.MAIN_W + 14, ly), cfg.COL_TEXT_DIM)
            draw_text(screen, font, value, (cfg.MAIN_W + 14, ly + 16), color)
        ly += 38

    pygame.draw.rect(screen, cfg.COL_PANEL_BG, (0, cfg.SCREEN_H - cfg.HUD_H, cfg.SCREEN_W, cfg.HUD_H))
    pygame.draw.line(screen, cfg.COL_PANEL_BORDER, (0, cfg.SCREEN_H - cfg.HUD_H), (cfg.SCREEN_W, cfg.SCREEN_H - cfg.HUD_H), 2)
    draw_text(screen, font_big, "AUTONOMOUS LANE-KEEPING SIMULATOR", (16, cfg.SCREEN_H - cfg.HUD_H + 10), (200, 210, 255))

    col1 = [
        f"FPS: {clock.get_fps():.0f}",
        f"Speed: {vehicle.v:5.1f} px/s   Target: {target_speed:5.1f}",
        f"Steering: {math.degrees(vehicle.delta):+5.1f} deg  (cmd {math.degrees(steer_cmd):+5.1f} deg)",
    ]
    col2 = [
        f"Lane error: {lane_result['lane_error']:+6.1f} px",
        f"True light: {world_node.system.state.upper():6s}   Detected: {light_state_detected.upper()}",
        f"Mode: {mode}",
    ]
    col3 = [
        "Controls: [ESC] Quit  [R] Reset",
        "[1/2/3] Force Red/Yellow/Green   [UP/DOWN] Speed",
        "[Q/A] Kp   [W/S] Kd   [E/D] Ki",
    ]
    for i, line in enumerate(col1):
        draw_text(screen, font, line, (16, cfg.SCREEN_H - cfg.HUD_H + 48 + i * 24))
    for i, line in enumerate(col2):
        draw_text(screen, font, line, (330, cfg.SCREEN_H - cfg.HUD_H + 48 + i * 24))
    for i, line in enumerate(col3):
        draw_text(screen, font_small, line, (640, cfg.SCREEN_H - cfg.HUD_H + 48 + i * 24), cfg.COL_TEXT_DIM)

    pygame.display.flip()


if __name__ == "__main__":
    main()