"""Telemetry HUD: fonts and text-drawing helpers."""

import pygame
from src import config as cfg


def make_hud_fonts():
    return (
        pygame.font.SysFont("consolas", 18),
        pygame.font.SysFont("consolas", 15),
        pygame.font.SysFont("consolas", 22, bold=True),
    )


def draw_text(surf, font, text, pos, color=cfg.COL_TEXT):
    surf.blit(font.render(text, True, color), pos)