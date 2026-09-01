"""Behavioral planner: converts detected traffic-light state + distance
to nearest intersection into a target speed and driving mode."""


class Planner:
    def __init__(self, stop_distance):
        self.stop_distance = stop_distance

    def plan(self, base_target_speed, nearest_dist, vehicle_v, light_state):
        target_speed = base_target_speed
        if nearest_dist < self.stop_distance and light_state == "red":
            target_speed = 0.0
            mode = "STOPPED" if vehicle_v < 3.0 else "BRAKING"
        elif nearest_dist < self.stop_distance and light_state == "yellow":
            target_speed = min(target_speed, base_target_speed * 0.4)
            mode = "BRAKING"
        else:
            mode = "FOLLOWING"
        return target_speed, mode