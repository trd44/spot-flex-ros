"""Reusable OWLv2 object detection helpers.

This module intentionally has no ROS imports so it can be used by both
offline tests and ROS action callbacks.
"""

from dataclasses import dataclass
import os
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import torch
from transformers import AutoModelForZeroShotObjectDetection, AutoProcessor


@dataclass(frozen=True)
class OwlDetection:
    """A single OWLv2 detection in image pixel coordinates."""

    label: str
    confidence: float
    bbox: Tuple[int, int, int, int]

    @property
    def center_pixel(self) -> Tuple[int, int]:
        x1, y1, x2, y2 = self.bbox
        return ((x1 + x2) // 2, (y1 + y2) // 2)


class OwlDetector:
    """Small wrapper around Hugging Face OWLv2 detection."""

    def __init__(
        self,
        model_id: str = "google/owlv2-base-patch16-ensemble",
        confidence_threshold: float = 0.15,
        device: Optional[str] = None,
        cache_dir: Optional[str] = None,
        local_files_only: bool = True,
    ) -> None:
        self.model_id = model_id
        self.confidence_threshold = confidence_threshold
        self.device = device or self._default_device()
        self.cache_dir = cache_dir or self._default_cache_dir()
        self.local_files_only = local_files_only
        self.model_ref = self._resolve_model_ref(model_id, self.cache_dir)
        self._processor = None
        self._model = None

    def detect(
        self,
        image_rgb: np.ndarray,
        labels: Sequence[str],
        confidence_threshold: Optional[float] = None,
    ) -> List[OwlDetection]:
        """Detect any of the requested labels in an RGB image."""
        if image_rgb is None or image_rgb.size == 0:
            raise ValueError("image_rgb is empty")
        if len(image_rgb.shape) != 3 or image_rgb.shape[2] != 3:
            raise ValueError("image_rgb must have shape HxWx3")
        if not labels:
            raise ValueError("labels must contain at least one text prompt")

        image_rgb = np.ascontiguousarray(image_rgb)
        self._load_model()
        threshold = confidence_threshold or self.confidence_threshold

        inputs = self._processor(
            text=list(labels),
            images=image_rgb,
            return_tensors="pt",
        ).to(self.device)

        with torch.no_grad():
            outputs = self._model(**inputs)

        target_sizes = torch.tensor([image_rgb.shape[:2]], device=self.device)
        if hasattr(self._processor, "post_process_object_detection"):
            results = self._processor.post_process_object_detection(
                outputs,
                threshold=threshold,
                target_sizes=target_sizes,
            )[0]
            text_labels = None
        else:
            results = self._processor.post_process_grounded_object_detection(
                outputs,
                threshold=threshold,
                target_sizes=target_sizes,
                text_labels=[list(labels)],
            )[0]
            text_labels = results.get("text_labels")

        detections = []
        for idx, (box, score, label_idx) in enumerate(zip(
            results["boxes"],
            results["scores"],
            results["labels"],
        )):
            x1, y1, x2, y2 = box.detach().cpu().numpy()
            label = (
                text_labels[idx]
                if text_labels is not None
                else labels[int(label_idx.detach().cpu())]
            )
            detections.append(
                OwlDetection(
                    label=label,
                    confidence=float(score.detach().cpu()),
                    bbox=(
                        int(round(x1)),
                        int(round(y1)),
                        int(round(x2)),
                        int(round(y2)),
                    ),
                )
            )

        return sorted(detections, key=lambda item: item.confidence, reverse=True)

    def best_detection(
        self,
        image_rgb: np.ndarray,
        labels: Sequence[str],
        confidence_threshold: Optional[float] = None,
    ) -> Optional[OwlDetection]:
        """Return the highest-confidence detection, if one exists."""
        detections = self.detect(image_rgb, labels, confidence_threshold)
        return detections[0] if detections else None

    def _load_model(self) -> None:
        if self._processor is not None and self._model is not None:
            return

        kwargs = {"local_files_only": self.local_files_only}
        if self.cache_dir and not Path(self.model_ref).exists():
            kwargs["cache_dir"] = self.cache_dir

        self._processor = AutoProcessor.from_pretrained(self.model_ref, **kwargs)
        self._model = AutoModelForZeroShotObjectDetection.from_pretrained(
            self.model_ref,
            **kwargs,
        ).to(self.device)
        self._model.eval()

    @staticmethod
    def _default_device() -> str:
        return "cuda" if torch.cuda.is_available() else "cpu"

    @staticmethod
    def _default_cache_dir() -> Optional[str]:
        candidates = [
            Path(os.environ["TRANSFORMERS_CACHE"])
            if os.environ.get("TRANSFORMERS_CACHE")
            else None,
            Path(os.environ["HF_HOME"]) / "hub"
            if os.environ.get("HF_HOME")
            else None,
            Path("/repo/workspace/model_cache/huggingface/hub"),
            Path("/repo/workspace/model_cache/huggingface"),
            Path("/opt/spot_flex_model_cache/huggingface/hub"),
            Path("/opt/spot_flex_model_cache/huggingface"),
            Path.cwd() / "model_cache" / "huggingface" / "hub",
            Path.cwd() / "model_cache" / "huggingface",
        ]
        for candidate in candidates:
            if candidate is not None and candidate.exists() and any(candidate.iterdir()):
                return str(candidate)
        return None

    @classmethod
    def _resolve_model_ref(cls, model_id: str, cache_dir: Optional[str]) -> str:
        if not cache_dir:
            return model_id

        cached_model_dir = Path(cache_dir) / f"models--{model_id.replace('/', '--')}"
        snapshot_root = cached_model_dir / "snapshots"
        if not snapshot_root.exists():
            return model_id

        ref_path = cached_model_dir / "refs" / "main"
        snapshot_ids = []
        if ref_path.exists():
            snapshot_ids.append(ref_path.read_text(encoding="utf-8").strip())
        snapshot_ids.extend(path.name for path in snapshot_root.iterdir() if path.is_dir())

        required_files = (
            "config.json",
            "preprocessor_config.json",
            "tokenizer_config.json",
            "vocab.json",
            "merges.txt",
        )
        weight_files = ("model.safetensors", "pytorch_model.bin")
        for snapshot_id in snapshot_ids:
            snapshot_dir = snapshot_root / snapshot_id
            if not snapshot_dir.exists():
                continue
            if all((snapshot_dir / name).exists() for name in required_files) and any(
                (snapshot_dir / name).exists() for name in weight_files
            ):
                return str(snapshot_dir)

        return model_id


def load_rgb_image(path: str) -> np.ndarray:
    """Load an image from disk as RGB."""
    image_bgr = cv2.imread(path, cv2.IMREAD_COLOR)
    if image_bgr is None:
        raise FileNotFoundError(path)
    return np.ascontiguousarray(cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB))


def draw_detections(
    image_rgb: np.ndarray,
    detections: Iterable[OwlDetection],
    output_path: str,
) -> None:
    """Save a simple debug image showing detections."""
    image_bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
    for detection in detections:
        x1, y1, x2, y2 = detection.bbox
        cx, cy = detection.center_pixel
        cv2.rectangle(image_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(image_bgr, (cx, cy), 5, (0, 0, 255), -1)
        cv2.putText(
            image_bgr,
            f"{detection.label} {detection.confidence:.2f}",
            (x1, max(20, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
    cv2.imwrite(output_path, image_bgr)
