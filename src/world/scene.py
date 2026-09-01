"""Static scene generation: background, buildings, road surface, lane
markings, and crosswalks. Rendered once and cached, since nothing here
changes frame-to-frame."""

import math
import random
import pygame
from src import config as cfg


def build_static_world(track):
    random.seed(cfg.RANDOM_SEED)
    surf = pygame.Surface((cfg.WORLD_W, cfg.WORLD_H))

    for row in range(0, cfg.WORLD_H, 4):
        f = row / cfg.WORLD_H
        col = tuple(int(a + (b - a) * f) for a, b in zip(cfg.COL_BG_TOP, cfg.COL_BG_BOT))
        pygame.draw.rect(surf, col, (0, row, cfg.WORLD_W, 4))

    stars = [
        (random.randint(0, cfg.WORLD_W), random.randint(0, cfg.WORLD_H), random.choice([1, 1, 1, 2]))
        for _ in range(260)
    ]
    for sx, sy, size in stars:
        pygame.draw.circle(surf, cfg.COL_STAR, (sx, sy), size)

    pygame.draw.circle(surf, (230, 230, 255), (cfg.WORLD_W - 160, 140), 70)
    pygame.draw.circle(surf, (18, 10, 46), (cfg.WORLD_W - 140, 120), 70)

    _draw_buildings(surf, track)
    _draw_road(surf, track)
    _draw_crosswalks(surf, track)
    _draw_obstacles(surf, track)

    return surf


def _draw_buildings(surf, track):
    sparse_track = track.samples[::8]
    for _ in range(70):
        bx = by = None
        for _ in range(30):
            cx = random.randint(40, cfg.WORLD_W - 40)
            cy = random.randint(40, cfg.WORLD_H - 40)
            nearest = min(math.hypot(cx - px, cy - py) for px, py in sparse_track)
            if nearest > cfg.ROAD_HALF_WIDTH + 55:
                bx, by = cx, cy
                break
        if bx is None:
            continue
        bw, bh = random.randint(28, 70), random.randint(60, 180)
        color = random.choice(cfg.COL_BUILDING)
        rect = pygame.Rect(bx - bw // 2, by - bh, bw, bh)
        pygame.draw.rect(surf, color, rect, border_radius=3)
        for wx in range(rect.left + 5, rect.right - 5, 10):
            for wy in range(rect.top + 8, rect.bottom - 6, 14):
                if random.random() < 0.6:
                    pygame.draw.rect(surf, cfg.COL_WINDOW, (wx, wy, 4, 6))


def _draw_road(surf, track):
    pygame.draw.polygon(surf, cfg.COL_ROAD, track.left_edge + track.right_edge[::-1])
    pygame.draw.lines(surf, cfg.COL_ROAD_EDGE, True, track.left_edge, 4)
    pygame.draw.lines(surf, cfg.COL_ROAD_EDGE, True, track.right_edge, 4)

    dash_len, gap_len = 14, 12
    dist_acc, dashing = 0.0, True
    n = len(track.samples)
    for i in range(n):
        p0, p1 = track.samples[i], track.samples[(i + 1) % n]
        seg_len = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        if dashing:
            pygame.draw.line(surf, cfg.COL_CENTER_LINE, p0, p1, 4)
        dist_acc += seg_len
        if dashing and dist_acc >= dash_len:
            dashing, dist_acc = False, 0.0
        elif not dashing and dist_acc >= gap_len:
            dashing, dist_acc = True, 0.0


def _draw_crosswalks(surf, track):
    for t in cfg.INTERSECTION_TS:
        cx, cy = track.point(t)
        tx, ty = track.tangent(t)
        nx, ny = -ty, tx
        for k in range(-4, 5, 2):
            off = k * 8
            p0 = (cx + tx * off - nx * cfg.ROAD_HALF_WIDTH, cy + ty * off - ny * cfg.ROAD_HALF_WIDTH)
            p1 = (cx + tx * off + nx * cfg.ROAD_HALF_WIDTH, cy + ty * off + ny * cfg.ROAD_HALF_WIDTH)
            pygame.draw.line(surf, (230, 230, 230), p0, p1, 5)


def _draw_obstacles(surf, track):
    for t in cfg.OBSTACLE_TS:
        cx, cy = track.point(t)
        tx, ty = track.tangent(t)
        nx, ny = -ty, tx
        ox = cx + nx * (cfg.ROAD_HALF_WIDTH - 10)
        oy = cy + ny * (cfg.ROAD_HALF_WIDTH - 10)
        pygame.draw.polygon(surf, (255, 140, 30), [(ox, oy - 10), (ox - 7, oy + 6), (ox + 7, oy + 6)])
        pygame.draw.line(surf, (255, 255, 255), (ox - 4, oy), (ox + 4, oy), 2)