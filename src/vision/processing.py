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

    # Morphological transformations to reduce noise
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Detect circles using Hough Circle Transform
    circles = cv2.HoughCircles(
        mask,
        cv2.HOUGH_GRADIENT,
        dp=1,
        minDist=20,
        param1=50,
        param2=30,
        minRadius=5,
        maxRadius=100,
    )

    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        return [(x, y, r) for x, y, r in circles]
    return []


def find_large_grey_wall(frame: VisionFrame, threshold: float = 0.8) -> bool:
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
