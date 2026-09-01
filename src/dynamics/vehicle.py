"""Kinematic bicycle (Ackermann) model:

    x_dot     = v * cos(theta)
    y_dot     = v * sin(theta)
    theta_dot = (v / L) * tan(delta)

Steering is rate-limited (STEER_RATE) and speed is governed by separate
accel/brake limits, so heading changes smoothly rather than snapping."""

import math
from src import config as cfg
from src.world.track import Track


class Vehicle:
    WHEELBASE = cfg.WHEELBASE
    MAX_SPEED = cfg.MAX_SPEED
    MAX_ACCEL = cfg.MAX_ACCEL
    MAX_BRAKE = cfg.MAX_BRAKE
    MAX_STEER = cfg.MAX_STEER
    STEER_RATE = cfg.STEER_RATE

    def __init__(self):
        self.reset()

    def reset(self):
        x, y = Track.point(0.0)
        tx, ty = Track.tangent(0.0)
        self.x, self.y = x, y
        self.theta = math.atan2(ty, tx)
        self.v = 40.0
        self.delta = 0.0

    def update(self, dt, target_speed, steer_cmd):
        steer_cmd = max(-self.MAX_STEER, min(self.MAX_STEER, steer_cmd))
        d_delta = steer_cmd - self.delta
        max_step = self.STEER_RATE * dt
        d_delta = max(-max_step, min(max_step, d_delta))
        self.delta += d_delta

        dv = target_speed - self.v
        step = min(dv, self.MAX_ACCEL * dt) if dv >= 0 else max(dv, -self.MAX_BRAKE * dt)
        self.v = max(0.0, self.v + step)

        self.theta += (self.v / self.WHEELBASE) * math.tan(self.delta) * dt
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt