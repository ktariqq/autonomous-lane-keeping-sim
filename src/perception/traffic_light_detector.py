"""RGB-threshold traffic-light color classifier over the upper region of
the camera frame. Deliberately independent from ground truth so
detection accuracy/latency can be inspected against it."""

import numpy as np


class TrafficLightDetector:
    def __init__(self, width, height):
        self.roi_bottom = int(height * 0.55)

    def process(self, frame_rgb):
        roi = np.ascontiguousarray(frame_rgb[0:self.roi_bottom, :, :])
        r = roi[..., 0].astype(np.int16)
        g = roi[..., 1].astype(np.int16)
        b = roi[..., 2].astype(np.int16)

        red_mask = (r > 180) & (g < 90) & (b < 90)
        yellow_mask = (r > 200) & (g > 170) & (b < 110)
        green_mask = (g > 180) & (r < 110) & (b < 160)

        counts = {
            "red": int(np.count_nonzero(red_mask)),
            "yellow": int(np.count_nonzero(yellow_mask)),
            "green": int(np.count_nonzero(green_mask)),
        }
        best = max(counts, key=counts.get)
        if counts[best] < 8:
            return "none", counts
        return best, counts