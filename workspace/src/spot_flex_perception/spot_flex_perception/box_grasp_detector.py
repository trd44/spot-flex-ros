"""Box grasp-point detection using OWLv2 followed by SAM.

This module has no ROS imports. The ROS node can pass an RGB image in and get
back an image-space grasp point.
"""

from dataclasses import dataclass
import os
from pathlib import Path
from typing import Optional, Sequence, Tuple

import cv2
import numpy as np
import torch
from segment_anything import SamPredictor, sam_model_registry

from spot_flex_perception.owl_detector import OwlDetection, OwlDetector


@dataclass(frozen=True)
class BoxGraspResult:
    """Result of the OWLv2 -> SAM box grasp pipeline."""

    owl_detection: OwlDetection
    grasp_pixel: Tuple[int, int]
    side: str
    rectangle: np.ndarray
    mask: np.ndarray


class BoxGraspDetector:
    """Find a left or right vertical-edge grasp point on a box."""

    def __init__(
        self,
        owl_detector: Optional[OwlDetector] = None,
        sam_checkpoint: Optional[str] = None,
        sam_model_type: str = "vit_b",
        device: Optional[str] = None,
        box_labels: Optional[Sequence[str]] = None,
    ) -> None:
        self.owl_detector = owl_detector or OwlDetector()
        self.sam_checkpoint = sam_checkpoint or self._default_sam_checkpoint()
        self.sam_model_type = sam_model_type
        self.device = device or self._default_device()
        self.box_labels = list(
            box_labels
            or [
                "cardboard box",
                "shipping box",
                "moving box",
                "corrugated box",
                "brown box",
            ]
        )
        self._sam_predictor = None

    def find_grasp_point(
        self,
        image_rgb: np.ndarray,
        side: str = "left",
    ) -> Optional[BoxGraspResult]:
        """Run OWLv2, refine the best box with SAM, and return an edge midpoint."""
        side = side.lower().strip()
        if side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'")

        owl_detection = self.owl_detector.best_detection(image_rgb, self.box_labels)
        if owl_detection is None:
            return None

        mask = self._segment_box(image_rgb, owl_detection.bbox)
        mask = _largest_component(mask)

        rectangle = _rectangle_from_mask(mask)
        if rectangle is None:
            return None

        ordered_rectangle = _order_box_points(rectangle)
        grasp_pixel = _edge_midpoint(ordered_rectangle, side)

        return BoxGraspResult(
            owl_detection=owl_detection,
            grasp_pixel=grasp_pixel,
            side=side,
            rectangle=ordered_rectangle,
            mask=mask,
        )

    def _segment_box(self, image_rgb: np.ndarray, bbox: Tuple[int, int, int, int]) -> np.ndarray:
        self._load_sam()
        self._sam_predictor.set_image(image_rgb)
        masks, scores, _ = self._sam_predictor.predict(
            box=np.array(bbox, dtype=np.float32),
            multimask_output=True,
        )
        best_idx = int(np.argmax(scores))
        return masks[best_idx].astype(np.uint8)

    def _load_sam(self) -> None:
        if self._sam_predictor is not None:
            return
        if not self.sam_checkpoint or not Path(self.sam_checkpoint).exists():
            raise FileNotFoundError(
                "SAM checkpoint not found. Expected: "
                f"{self.sam_checkpoint or '<unset>'}"
            )

        sam = sam_model_registry[self.sam_model_type](
            checkpoint=self.sam_checkpoint,
        ).to(self.device)
        self._sam_predictor = SamPredictor(sam)

    @staticmethod
    def _default_device() -> str:
        return "cuda" if torch.cuda.is_available() else "cpu"

    @staticmethod
    def _default_sam_checkpoint() -> Optional[str]:
        model_dir = os.environ.get("SPOT_FLEX_MODEL_DIR")
        candidates = [
            Path(model_dir) / "sam" / "sam_vit_b_01ec64.pth"
            if model_dir
            else None,
            Path("/repo/workspace/model_cache/sam/sam_vit_b_01ec64.pth"),
            Path("/opt/spot_flex_model_cache/sam/sam_vit_b_01ec64.pth"),
            Path.cwd() / "model_cache" / "sam" / "sam_vit_b_01ec64.pth",
        ]
        for candidate in candidates:
            if candidate is not None and candidate.exists():
                return str(candidate)
        return None


def draw_box_grasp_result(
    image_rgb: np.ndarray,
    result: BoxGraspResult,
    output_path: str,
) -> None:
    """Save a debug image showing the SAM mask, rectangle, and grasp point."""
    image_bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
    overlay = image_bgr.copy()
    overlay[result.mask > 0] = (0, 255, 0)
    image_bgr = cv2.addWeighted(image_bgr, 0.65, overlay, 0.35, 0)

    x1, y1, x2, y2 = result.owl_detection.bbox
    cv2.rectangle(image_bgr, (x1, y1), (x2, y2), (255, 0, 0), 2)

    rect = result.rectangle.astype(np.int32)
    cv2.polylines(image_bgr, [rect], True, (0, 0, 255), 2, cv2.LINE_AA)
    for idx, (x, y) in enumerate(rect):
        cv2.circle(image_bgr, (int(x), int(y)), 4, (255, 0, 255), -1)
        cv2.putText(
            image_bgr,
            f"P{idx}",
            (int(x) + 4, int(y) - 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 0, 255),
            1,
            cv2.LINE_AA,
        )

    gx, gy = result.grasp_pixel
    cv2.circle(image_bgr, (gx, gy), 8, (0, 0, 255), -1)
    cv2.putText(
        image_bgr,
        f"{result.side} grasp",
        (gx + 8, gy - 8),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 0, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.imwrite(output_path, image_bgr)


def _largest_component(mask: np.ndarray) -> np.ndarray:
    mask_u8 = (mask > 0).astype(np.uint8)
    count, labels = cv2.connectedComponents(mask_u8)
    if count <= 1:
        return mask_u8

    largest_label = 1 + np.argmax(
        [(labels == label).sum() for label in range(1, count)]
    )
    return (labels == largest_label).astype(np.uint8)


def _rectangle_from_mask(mask: np.ndarray) -> Optional[np.ndarray]:
    contours, _ = cv2.findContours(
        (mask > 0).astype(np.uint8),
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    if not contours:
        return None

    contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(contour) < 10:
        return None

    epsilon = 0.02 * cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, epsilon, True)
    if len(approx) == 4:
        return approx.reshape(-1, 2).astype(np.float32)

    rect = cv2.minAreaRect(contour)
    return cv2.boxPoints(rect).astype(np.float32)


def _order_box_points(box: np.ndarray) -> np.ndarray:
    center = box.mean(axis=0)
    angles = np.arctan2(box[:, 1] - center[1], box[:, 0] - center[0])
    return box[np.argsort(angles)]


def _edge_midpoint(rectangle: np.ndarray, side: str) -> Tuple[int, int]:
    if side == "left":
        midpoint = (rectangle[0] + rectangle[3]) / 2.0
    else:
        midpoint = (rectangle[1] + rectangle[2]) / 2.0
    return int(round(midpoint[0])), int(round(midpoint[1]))
