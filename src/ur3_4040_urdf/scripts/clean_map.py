#!/usr/bin/env python3
"""
SLAM으로 저장한 occupancy grid (.pgm)에서 isolated stale obstacle 군집 제거.

큰 군집(벽)은 보존, 작은 군집(lidar self-hit, noise 잔재)만 free로 변환.

Usage:
  ros2 run ur3_4040_urdf clean_map.py <input.pgm> [-t 20] [-o output.pgm] [--no-backup]

Examples:
  # 기존 파일 덮어쓰기 (자동 .bak 생성)
  ros2 run ur3_4040_urdf clean_map.py ~/ur3/src/ur3_4040_urdf/maps/my_capstone_map.pgm

  # 다른 파일로 출력
  ros2 run ur3_4040_urdf clean_map.py input.pgm -o cleaned.pgm

  # threshold 변경 (10 픽셀 미만 군집만 제거 = 더 보수적)
  ros2 run ur3_4040_urdf clean_map.py input.pgm -t 10
"""
import argparse
import os
import shutil
import sys

import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser(
        description="SLAM saved map의 isolated stale obstacle 군집 제거",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="큰 군집(벽)은 보존, 작은 군집만 free로 변환합니다.",
    )
    parser.add_argument("input", help="입력 pgm 파일 경로")
    parser.add_argument(
        "-o", "--output", default=None,
        help="출력 pgm 경로 (생략 시 input 덮어쓰기 + .bak 백업)",
    )
    parser.add_argument(
        "-t", "--threshold", type=int, default=20,
        help="이 픽셀 수 미만의 occupied 군집을 free로 변환 (default 20)",
    )
    parser.add_argument(
        "--no-backup", action="store_true",
        help="input 덮어쓸 때 .bak 백업 생성 안 함",
    )
    args = parser.parse_args()

    if not os.path.isfile(args.input):
        sys.exit(f"[ERROR] 입력 파일 없음: {args.input}")

    output = args.output if args.output else args.input
    overwriting = (os.path.abspath(output) == os.path.abspath(args.input))

    img = cv2.imread(args.input, cv2.IMREAD_UNCHANGED)
    if img is None:
        sys.exit(f"[ERROR] 이미지 읽기 실패: {args.input}")
    if img.dtype != np.uint8:
        sys.exit(f"[ERROR] 8-bit greyscale 아님: dtype={img.dtype}")

    h, w = img.shape
    print(f"입력: {args.input} ({w}x{h})")

    occupied_mask = (img < 50).astype(np.uint8)
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
        occupied_mask, connectivity=8
    )
    n_clusters = num_labels - 1
    sizes = [stats[i, cv2.CC_STAT_AREA] for i in range(1, num_labels)]
    print(f"  occupied 군집: {n_clusters}개")
    if sizes:
        sizes_sorted = sorted(sizes)
        print(
            f"  크기 분포: min={sizes_sorted[0]}, "
            f"median={sizes_sorted[len(sizes_sorted)//2]}, "
            f"max={sizes_sorted[-1]}"
        )

    clean = img.copy()
    removed_clusters, removed_pixels = 0, 0
    for i in range(1, num_labels):
        size = stats[i, cv2.CC_STAT_AREA]
        if size < args.threshold:
            clean[labels == i] = 254  # free (white)
            removed_clusters += 1
            removed_pixels += size

    print(f"제거: {removed_clusters}개 군집 ({removed_pixels} 픽셀, threshold < {args.threshold})")
    print(f"보존: {n_clusters - removed_clusters}개 군집 (벽 등 큰 군집)")

    if overwriting and not args.no_backup:
        bak = args.input + ".bak"
        shutil.copy2(args.input, bak)
        print(f"백업: {bak}")

    cv2.imwrite(output, clean)
    print(f"저장: {output}")


if __name__ == "__main__":
    main()
