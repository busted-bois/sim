from __future__ import annotations

import cv2
import numpy as np


def mean_rgb_summary(image_rgb: np.ndarray) -> str:
    """Short log string with per-channel mean intensity (OpenCV `cv2.mean`)."""
    if image_rgb.ndim != 3 or image_rgb.shape[2] != 3:
        return ""
    m = cv2.mean(image_rgb)
    return f" cv_mean=({m[0]:.1f},{m[1]:.1f},{m[2]:.1f})"
