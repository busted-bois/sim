import cv2
import numpy as np

from src.vision.feed import VisionFrame


def find_red_circles(frame: VisionFrame) -> list[tuple[int, int, int]]:
    """
    Finds red circles in a vision frame.

    Args:
        frame: The vision frame to process.

    Returns:
        A list of circles, where each circle is a tuple of (x, y, radius).
    """
    hsv = cv2.cvtColor(frame.image_rgb, cv2.COLOR_RGB2HSV)

    # Define range for red color and create a mask
    lower_red = np.array([0, 120, 70])
    upper_red = np.array([10, 255, 255])
    mask1 = cv2.inRange(hsv, lower_red, upper_red)

    lower_red = np.array([170, 120, 70])
    upper_red = np.array([180, 255, 255])
    mask2 = cv2.inRange(hsv, lower_red, upper_red)

    mask = mask1 + mask2

    # Lighter morphology so distant (small) targets survive the clean-up.
    mask = cv2.erode(mask, None, iterations=1)
    mask = cv2.dilate(mask, None, iterations=2)

    # Detect circles using Hough Circle Transform. Lower param2 + minRadius
    # makes the detector pick up small/faint circles at distance, which is
    # what we want for finding a far-away red target before it's already
    # under the drone.
    circles = cv2.HoughCircles(
        mask,
        cv2.HOUGH_GRADIENT,
        dp=1,
        minDist=15,
        param1=50,
        param2=18,
        minRadius=2,
        maxRadius=120,
    )

    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        return [(x, y, r) for x, y, r in circles]
    return []


def find_large_grey_wall(frame: VisionFrame, threshold: float = 0.95) -> bool:
    """
    Detects if a large grey wall is in the frame.

    Args:
        frame: The vision frame to process.
        threshold: The percentage of the image that must be grey to be considered a wall.

    Returns:
        True if a large grey wall is detected, False otherwise.
    """
    gray_image = cv2.cvtColor(frame.image_rgb, cv2.COLOR_RGB2GRAY)

    # Define range for grey color
    lower_grey = 100
    upper_grey = 200
    mask = cv2.inRange(gray_image, lower_grey, upper_grey)

    # Calculate the percentage of the image that is grey
    grey_percentage = np.sum(mask > 0) / (frame.width * frame.height)

    return grey_percentage > threshold


def red_target_offset_normalized(frame: VisionFrame) -> tuple[float, float] | None:
    """Largest red circle vs image center, normalized to roughly [-1, 1].

    Returns None if no circle; (nx, ny) where positive nx means target is right of center.
    """
    info = red_target_info_normalized(frame)
    if info is None:
        return None
    return (info[0], info[1])


def red_target_info_normalized(frame: VisionFrame) -> tuple[float, float, float] | None:
    """Largest red circle as (nx, ny, r_frac).

    nx, ny in roughly [-1, 1] against image center. r_frac is the circle radius
    divided by min(width, height) — a rough "how much of the frame does the
    target occupy" proxy that grows as the drone approaches.
    """
    circles = find_red_circles(frame)
    if not circles:
        return None
    x, y, r = max(circles, key=lambda c: c[2])
    w, h = float(frame.width), float(frame.height)
    if w <= 0 or h <= 0:
        return None
    nx = (x - 0.5 * w) / (0.5 * w)
    ny = (y - 0.5 * h) / (0.5 * h)
    r_frac = float(r) / max(1.0, min(w, h))
    return (float(nx), float(ny), r_frac)


def grey_wall_info_normalized(
    frame: VisionFrame,
    min_fraction: float = 0.05,
    edge_margin_frac: float = 0.02,
) -> tuple[float, float, float] | None:
    """Largest interior connected grey blob as (nx, ny, r_frac).

    Filters out the ground (bottom edge) and sky (top edge) by rejecting any
    blob whose bounding box touches the top or bottom of the frame — a real
    wall should sit inside the frame, above the ground and below the sky.

    Then picks the largest remaining connected component. Returns its centroid
    normalized to image center and its area as a fraction of the frame, or
    None if no interior blob covers at least `min_fraction` of the frame.

    nx, ny are in roughly [-1, 1]. r_frac is the blob's area fraction.
    """
    gray = cv2.cvtColor(frame.image_rgb, cv2.COLOR_RGB2GRAY)
    lower_grey = 110
    upper_grey = 180
    mask = cv2.inRange(gray, lower_grey, upper_grey)
    # Erode first to break ground/sky apart from walls where they touch at the
    # horizon; then dilate to restore wall mass.
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    w, h = float(frame.width), float(frame.height)
    total_px = w * h
    if total_px <= 0:
        return None

    num_labels, _labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
    if num_labels <= 1:
        return None

    bottom_limit = int(h - edge_margin_frac * h)

    best_label = 0
    best_area = 0
    for label in range(1, num_labels):
        top = int(stats[label, cv2.CC_STAT_TOP])
        height = int(stats[label, cv2.CC_STAT_HEIGHT])
        bottom = top + height
        # Reject blobs clinging to the bottom of the frame (that's the ground).
        # We deliberately allow blobs to touch the top — a wall approaching the
        # drone commonly clips the top edge of the frame once close, and we
        # still want to track it.
        if bottom >= bottom_limit:
            continue
        # Reject blobs that stretch almost the full width — that's usually sky,
        # horizon haze, or a distant wall-of-everything.
        width_ext = int(stats[label, cv2.CC_STAT_WIDTH])
        if width_ext >= int(0.95 * w):
            continue
        area = int(stats[label, cv2.CC_STAT_AREA])
        if area > best_area:
            best_area = area
            best_label = label

    if best_label == 0:
        return None

    blob_fraction = best_area / total_px
    if blob_fraction < min_fraction:
        return None

    cx, cy = centroids[best_label]
    nx = (float(cx) - 0.5 * w) / (0.5 * w)
    ny = (float(cy) - 0.5 * h) / (0.5 * h)
    return (nx, ny, float(blob_fraction))
