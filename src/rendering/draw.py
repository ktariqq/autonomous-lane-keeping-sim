"""Dynamic overlay rendering: traffic-light poles and the vehicle sprite.
Kept separate from the static scene since these repaint every frame."""

import math
import pygame
from src import config as cfg


def draw_traffic_lights(surf, track, state):
    colors = {"red": cfg.RED, "yellow": cfg.YELLOW, "green": cfg.GREEN}
    order = ["red", "yellow", "green"]
    for t in cfg.INTERSECTION_TS:
        cx, cy = track.point(t)
        tx, ty = track.tangent(t)
        nx, ny = -ty, tx
        px = cx + nx * (cfg.ROAD_HALF_WIDTH + 30)
        py = cy + ny * (cfg.ROAD_HALF_WIDTH + 30)
        pygame.draw.rect(surf, (40, 40, 55), (px - 6, py - 46, 12, 60), border_radius=3)
        for i, name in enumerate(order):
            lit = name == state
            col = colors[name] if lit else cfg.DIM
            cyi = py - 40 + i * 16
            pygame.draw.circle(surf, col, (int(px), int(cyi)), 6)
            if lit:
                glow = pygame.Surface((28, 28), pygame.SRCALPHA)
                pygame.draw.circle(glow, (*colors[name], 90), (14, 14), 13)
                surf.blit(glow, (px - 14, cyi - 14), special_flags=pygame.BLEND_ADD)


def draw_vehicle(surf, vehicle, off_x, off_y):
    """Only ever drawn on the display surface, never on the surface the
    camera samples from — the vehicle must not see itself."""
    sx, sy = vehicle.x + off_x, vehicle.y + off_y
    length, width = 26, 14
    cos_t, sin_t = math.cos(vehicle.theta), math.sin(vehicle.theta)

    def rot(lx, ly):
        return (sx + lx * cos_t - ly * sin_t, sy + lx * sin_t + ly * cos_t)

    corners = [rot(length / 2, -width / 2), rot(length / 2, width / 2),
               rot(-length / 2, width / 2), rot(-length / 2, -width / 2)]
    pygame.draw.polygon(surf, cfg.COL_VEHICLE, corners)
    pygame.draw.polygon(surf, cfg.COL_VEHICLE_TRIM, corners, 2)

    hl1, hl2 = rot(length / 2, -width / 3), rot(length / 2, width / 3)
    pygame.draw.circle(surf, (255, 255, 200), (int(hl1[0]), int(hl1[1])), 3)
    pygame.draw.circle(surf, (255, 255, 200), (int(hl2[0]), int(hl2[1])), 3)

    front = rot(length / 2, 0)
    wheel_dir = vehicle.theta + vehicle.delta
    tip = (front[0] + 14 * math.cos(wheel_dir), front[1] + 14 * math.sin(wheel_dir))
    pygame.draw.line(surf, (255, 255, 0), front, tip, 2)