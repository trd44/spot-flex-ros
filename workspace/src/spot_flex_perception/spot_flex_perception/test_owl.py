"""Offline test for OWLv2 detection on package test images."""

from pathlib import Path

from spot_flex_perception.owl_detector import (
    OwlDetector,
    draw_detections,
    load_rgb_image,
)


def main() -> None:
    workspace_dir = Path("/repo/workspace")
    image_path = (
        workspace_dir
        / "src"
        / "spot_flex_perception"
        / "test_images"
        / "cardboard_box.jpg"
    )
    output_path = (
        workspace_dir
        / "src"
        / "spot_flex_perception"
        / "test_images"
        / "cardboard_box_owl_debug.jpg"
    )

    labels = ["cardboard box", "shipping box", "moving box", 
              "corrugated box", "brown box"]
    image_rgb = load_rgb_image(str(image_path))
    detector = OwlDetector(confidence_threshold=0.3, local_files_only=True)
    detections = detector.detect(image_rgb, labels)
    best_detection = detector.best_detection(image_rgb, labels)

    print(f"image={image_path}")
    print(f"raw_detections={len(detections)}")

    if best_detection:
        print(
            f"best_detection label={best_detection.label!r} "
            f"score={best_detection.confidence:.3f} "
            f"bbox={best_detection.bbox} "
            f"center={best_detection.center_pixel}"
        )
        draw_detections(image_rgb, [best_detection], str(output_path))
        print(f"saved_debug_image={output_path}")
    else:
        print("No detections found. Try lowering the confidence threshold.")


if __name__ == "__main__":
    main()
