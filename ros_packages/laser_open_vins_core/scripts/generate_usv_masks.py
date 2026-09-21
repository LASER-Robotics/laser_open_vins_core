#!/usr/bin/env python3
"""Generate the same provisional OpenVINS exclusion mask for both USV profiles."""

import argparse
from pathlib import Path

import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--water-start-row", type=int, required=True,
        help="First excluded row (0-based); rows below it are also excluded.")
    parser.add_argument(
        "--params-dir", type=Path,
        default=Path(__file__).resolve().parents[1] / "params",
        help="Source params directory containing realsense_usv and realsense_usv_mono.")
    args = parser.parse_args()

    # Validate every camera before overwriting either mask. Read the actual
    # calibration, so a resolution change cannot silently create a wrong mask.
    profiles = [args.params_dir / name for name in ("realsense_usv", "realsense_usv_mono")]
    sizes = []
    for profile, count in zip(profiles, (2, 1)):
        calibration = cv2.FileStorage(str(profile / "kalibr_imucam_chain.yaml"), cv2.FILE_STORAGE_READ)
        if not calibration.isOpened():
            parser.error(f"Cannot read calibration in {profile}")
        try:
            for index in range(count):
                resolution = calibration.getNode(f"cam{index}").getNode("resolution")
                if resolution.size() != 2:
                    parser.error(f"Missing cam{index} resolution in {profile}")
                sizes.append(tuple(int(resolution.at(i).real()) for i in range(2)))
        finally:
            calibration.release()
    if len(set(sizes)) != 1:
        parser.error("This shared mask requires the same resolution in all cameras")
    width, height = sizes[0]
    if width <= 0 or not 0 < args.water_start_row < height:
        parser.error(f"--water-start-row must be between 1 and {height - 1}")

    mask = np.zeros((height, width), dtype=np.uint8)
    mask[args.water_start_row:, :] = 255  # White means EXCLUDE in OpenVINS.
    for profile in profiles:
        target = profile / "mask_water.png"
        if not cv2.imwrite(str(target), mask):
            raise RuntimeError(f"Cannot write {target}")
        print(f"{target}: {width}x{height}, exclude y >= {args.water_start_row}")
    print("Validate the crop on the camera images throughout the trajectory; reinstall the package.")


if __name__ == "__main__":
    main()
