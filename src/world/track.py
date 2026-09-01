"""Closed parametric track geometry: centerline, tangents, and road edges."""

import math
from src import config as cfg


class Track:
    def __init__(self):
        self.samples = [self.point(i / cfg.N_SAMPLES) for i in range(cfg.N_SAMPLES)]
        self.tangents = [self.tangent(i / cfg.N_SAMPLES) for i in range(cfg.N_SAMPLES)]

        self.left_edge, self.right_edge = [], []
        for (px, py), (tx, ty) in zip(self.samples, self.tangents):
            nx, ny = -ty, tx
            self.left_edge.append((px + nx * cfg.ROAD_HALF_WIDTH, py + ny * cfg.ROAD_HALF_WIDTH))
            self.right_edge.append((px - nx * cfg.ROAD_HALF_WIDTH, py - ny * cfg.ROAD_HALF_WIDTH))

        self.intersection_positions = [self.point(t) for t in cfg.INTERSECTION_TS]

    @staticmethod
    def point(t):
        theta = t * 2 * math.pi
        r_mod = 1.0 + cfg.TRACK_WIGGLE * math.sin(cfg.TRACK_WIGGLE_K * theta)
        x = cfg.WORLD_CX + cfg.TRACK_RX * r_mod * math.cos(theta)
        y = cfg.WORLD_CY + cfg.TRACK_RY * r_mod * math.sin(theta)
        return x, y

    @classmethod
    def tangent(cls, t, eps=1e-4):
        x0, y0 = cls.point((t - eps) % 1.0)
        x1, y1 = cls.point((t + eps) % 1.0)
        dx, dy = x1 - x0, y1 - y0
        n = math.hypot(dx, dy) or 1e-6
        return dx / n, dy / n

    def nearest_intersection_distance(self, x, y):
        return min(math.hypot(x - ix, y - iy) for ix, iy in self.intersection_positions)