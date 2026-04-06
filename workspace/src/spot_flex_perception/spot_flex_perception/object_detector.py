"""
Object detection utilities

Wraps ML models (OWL-ViT, YOLO) for detecting objects, cabinets,
and clear surfaces in images.
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Optional, Tuple


@dataclass
class Detection:
    """A detected object in image space."""
    label: str
    confidence: float
    bbox: Tuple[int, int, int, int]  # x1, y1, x2, y2
    center_pixel: Tuple[int, int]


class ObjectDetector:
    """
    Runs object detection on images.

    Uses OWL-ViT for open-vocabulary detection and optionally
    YOLO for faster known-object detection.
    """

    def __init__(self, model_name: str = "owl-vit"):
        self.model_name = model_name
        self._model = None

    def _load_model(self):
        """Load the detection model on first use."""
        if self._model is not None:
            return

        if self.model_name == "owl-vit":
            pass
        elif self.model_name == "yolo":
            pass

    def detect(self, image: np.ndarray, target_label: str) -> List[Detection]:
        """
        Detect instances of target_label in the image.

        Args:
            image: RGB image as numpy array (H, W, 3)
            target_label: What to look for (e.g., "mug", "cabinet handle")

        Returns:
            List of Detection results, sorted by confidence descending
        """
        self._load_model()
        return []

    def detect_cabinet(self, image: np.ndarray) -> Optional[Detection]:
        """
        Detect the custom-built cabinet in the image.

        Uses a specialized classifier since the cabinet may not look
        like a standard cabinet to a general-purpose detector.
        """
        self._load_model()
        return None

    def find_clear_surface(
        self, image: np.ndarray, depth: np.ndarray
    ) -> Optional[Tuple[int, int]]:
        """
        Find a clear spot on a surface to place an object.

        Args:
            image: RGB image
            depth: Aligned depth image (same resolution)

        Returns:
            (u, v) pixel coordinates of a clear placement spot, or None
        """
        return None
