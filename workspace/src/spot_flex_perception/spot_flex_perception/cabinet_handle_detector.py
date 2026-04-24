"""Cabinet handle detection using a simple green color blob.

This module has no ROS imports so the same detector can be used by offline
tests and action callbacks.
"""

from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np


@dataclass(frozen=True)
class CabinetHandleResult:
    """Detected handle center in image pixel coordinates."""

    center_pixel: Tuple[int, int]
    bbox: Tuple[int, int, int, int]
    area: float
    mask: np.ndarray


class CabinetHandleDetector:
    """Detect a green handle by thresholding in HSV color space."""

    def __init__(
        self,
        lower_hsv: Tuple[int, int, int] = (35, 60, 40),
        upper_hsv: Tuple[int, int, int] = (90, 255, 255),
        min_area: float = 50.0,
    ) -> None:
        self.lower_hsv = np.array(lower_hsv, dtype=np.uint8)
        self.upper_hsv = np.array(upper_hsv, dtype=np.uint8)
        self.min_area = min_area

    def detect(self, image_rgb: np.ndarray) -> Optional[CabinetHandleResult]:
        """Return the largest green blob center, if present."""
        if image_rgb is None or image_rgb.size == 0:
            raise ValueError("image_rgb is empty")
        if len(image_rgb.shape) != 3 or image_rgb.shape[2] != 3:
            raise ValueError("image_rgb must have shape HxWx3")

        image_bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        contour = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(contour))
        if area < self.min_area:
            return None

        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            return None

        cx = int(moments["m10"] / moments["m00"])
        cy = int(moments["m01"] / moments["m00"])
        x, y, w, h = cv2.boundingRect(contour)

        return CabinetHandleResult(
            center_pixel=(cx, cy),
            bbox=(x, y, x + w, y + h),
            area=area,
            mask=mask,
        )


def draw_cabinet_handle_result(
    image_rgb: np.ndarray,
    result: CabinetHandleResult,
    output_path: str,
) -> None:
    """Save a debug image showing the detected green blob."""
    image_bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
    overlay = image_bgr.copy()
    overlay[result.mask > 0] = (0, 255, 0)
    image_bgr = cv2.addWeighted(image_bgr, 0.7, overlay, 0.3, 0)

    x1, y1, x2, y2 = result.bbox
    cx, cy = result.center_pixel
    cv2.rectangle(image_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.circle(image_bgr, (cx, cy), 7, (0, 0, 255), -1)
    cv2.putText(
        image_bgr,
        f"handle ({cx}, {cy})",
        (cx + 8, cy - 8),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 0, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.imwrite(output_path, image_bgr)
