"""Simulated front-facing camera.

Builds a forward view by applying an OpenCV affine warp to the top-down
world scene, rotating/translating so the vehicle's heading points "up"
and the vehicle sits near the bottom of the frame. This produces a real,
physically grounded image a color-threshold CV pipeline can run against,
without a full 3D renderer."""

import math
import cv2
from src import config as cfg


class Camera:
    def __init__(self):
        self.width = cfg.CAM_W
        self.height = cfg.CAM_H
        self.origin_x = self.width / 2
        self.origin_y = self.height - 30

    def capture(self, world_bgr, vehicle):
        angle_deg = 90.0 + math.degrees(vehicle.theta)
        M = cv2.getRotationMatrix2D((vehicle.x, vehicle.y), angle_deg, 1.0)
        M[0, 2] += self.origin_x - vehicle.x
        M[1, 2] += self.origin_y - vehicle.y

        frame_bgr = cv2.warpAffine(
            world_bgr, M, (self.width, self.height), borderValue=(30, 12, 8)
        )
        return cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)