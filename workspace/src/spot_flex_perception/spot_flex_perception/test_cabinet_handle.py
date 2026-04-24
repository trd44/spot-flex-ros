"""Offline test for green cabinet handle detection."""

from pathlib import Path

from spot_flex_perception.cabinet_handle_detector import (
    CabinetHandleDetector,
    draw_cabinet_handle_result,
)
from spot_flex_perception.owl_detector import load_rgb_image


def main() -> None:
    workspace_dir = Path("/repo/workspace")
    image_path = (
        workspace_dir
        / "src"
        / "spot_flex_perception"
        / "test_images"
        / "test_cabinet.jpg"
    )
    output_path = (
        workspace_dir
        / "src"
        / "spot_flex_perception"
        / "test_images"
        / "test_cabinet_handle_debug.jpg"
    )

    image_rgb = load_rgb_image(str(image_path))
    detector = CabinetHandleDetector()
    result = detector.detect(image_rgb)

    print(f"image={image_path}")
    if result is None:
        print("found=False")
        return

    print("found=True")
    print(f"center_pixel={result.center_pixel}")
    print(f"bbox={result.bbox}")
    print(f"area={result.area:.1f}")
    draw_cabinet_handle_result(image_rgb, result, str(output_path))
    print(f"saved_debug_image={output_path}")


if __name__ == "__main__":
    main()
