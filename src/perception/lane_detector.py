"""Classical CV lane-finding: color thresholding over a fixed image band,
reduced to a single lane-center pixel estimate and a signed error term."""

import numpy as np


class LaneDetector:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.band_top = int(height * 0.62)
        self.band_bottom = int(height * 0.92)
        self.center_x = width / 2.0
        self.last_center = self.center_x

    def process(self, frame_rgb):
        band = np.ascontiguousarray(frame_rgb[self.band_top:self.band_bottom, :, :])

        white_mask = np.all(band > 200, axis=2)

        r = band[..., 0].astype(np.int16)
        g = band[..., 1].astype(np.int16)
        b = band[..., 2].astype(np.int16)
        cyan_mask = (g > 150) & (b > 150) & (r < 120)

        cols_cyan = np.where(cyan_mask.any(axis=0))[0]
        cols_white = np.where(white_mask.any(axis=0))[0]

        if len(cols_cyan) > 3:
            center, detected = float(np.mean(cols_cyan)), True
        elif len(cols_white) > 3:
            center, detected = float(np.mean(cols_white)), True
        else:
            center, detected = self.last_center, False

        self.last_center = center
        lane_error = center - self.center_x

        return {"detected": detected, "center_x": center, "lane_error": lane_error}