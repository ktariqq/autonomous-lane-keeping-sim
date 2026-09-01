"""Central configuration: screen/world dimensions, colors, and default
tuning parameters. Kept separate from logic so tuning does not require
touching pipeline code."""

import math

FPS = 60

MAIN_W, MAIN_H = 860, 700
CAM_W, CAM_H = 320, 240
CAM_DISPLAY_SCALE = 1.4
HUD_H = 190
SIDE_W = 360
SCREEN_W = MAIN_W + SIDE_W
SCREEN_H = MAIN_H + HUD_H

WORLD_W, WORLD_H = 1700, 1300
WORLD_CX, WORLD_CY = WORLD_W // 2, WORLD_H // 2

TRACK_RX, TRACK_RY = 620, 460
TRACK_WIGGLE = 0.14
TRACK_WIGGLE_K = 3
ROAD_HALF_WIDTH = 46
N_SAMPLES = 480

INTERSECTION_TS = [0.0, 0.5]
OBSTACLE_TS = [0.15, 0.72]

# --- Colors ---------------------------------------------------------------
COL_BG_TOP = (8, 6, 28)
COL_BG_BOT = (18, 10, 46)
COL_STAR = (235, 235, 255)
COL_ROAD = (58, 58, 70)
COL_ROAD_EDGE = (255, 255, 255)     # lane-boundary color; perception target
COL_CENTER_LINE = (0, 255, 255)     # center-line color; perception target
COL_BUILDING = [(70, 40, 110), (40, 70, 120), (90, 50, 70), (35, 90, 95)]
COL_WINDOW = (255, 220, 120)
COL_VEHICLE = (255, 90, 90)
COL_VEHICLE_TRIM = (255, 230, 230)
COL_TEXT = (225, 230, 245)
COL_TEXT_DIM = (140, 145, 170)
COL_PANEL_BG = (14, 12, 34)
COL_PANEL_BORDER = (90, 90, 130)

RED = (235, 40, 40)
YELLOW = (250, 210, 40)
GREEN = (60, 230, 110)
DIM = (70, 70, 80)

# --- Vehicle dynamics -------------------------------------------------------
WHEELBASE = 34.0
MAX_SPEED = 160.0
MAX_ACCEL = 90.0
MAX_BRAKE = 220.0
MAX_STEER = math.radians(32)
STEER_RATE = math.radians(140)

# --- Control / planning -----------------------------------------------------
PID_KP, PID_KI, PID_KD = 2.2, 0.015, 0.9
PID_I_LIMIT = 400.0
BASE_TARGET_SPEED = 70.0
STOP_DISTANCE = 130.0

RANDOM_SEED = 7