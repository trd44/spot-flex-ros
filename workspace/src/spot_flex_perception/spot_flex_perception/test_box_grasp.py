"""Offline test for the OWLv2 -> SAM box grasp pipeline."""

from pathlib import Path

from spot_flex_perception.box_grasp_detector import (
    BoxGraspDetector,
    draw_box_grasp_result,
)
from spot_flex_perception.owl_detector import load_rgb_image


def main() -> None:
    workspace_dir = Path("/repo/workspace")
    image_path = (
        workspace_dir
        / "src"
        / "spot_flex_perception"
        / "test_images"
        / "cardboard_box.jpg"
    )

    image_rgb = load_rgb_image(str(image_path))
    detector = BoxGraspDetector()

    for side in ("left", "right"):
        result = detector.find_grasp_point(image_rgb, side=side)
        output_path = (
            workspace_dir
            / "src"
            / "spot_flex_perception"
            / "test_images"
            / f"cardboard_box_grasp_{side}_debug.jpg"
        )

        print(f"image={image_path}")
        print(f"side={side}")
        if result is None:
            print("found=False")
            continue

        print("found=True")
        print(
            f"owl label={result.owl_detection.label!r} "
            f"score={result.owl_detection.confidence:.3f} "
            f"bbox={result.owl_detection.bbox}"
        )
        print(f"grasp_pixel={result.grasp_pixel}")
        print(f"rectangle={result.rectangle.round(1).tolist()}")
        draw_box_grasp_result(image_rgb, result, str(output_path))
        print(f"saved_debug_image={output_path}")


if __name__ == "__main__":
    main()
