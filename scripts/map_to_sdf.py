#!/usr/bin/env python3
"""
map_to_sdf.py — nav2 점유격자 맵(.pgm + .yaml)을 Gazebo(gz sim) SDF 월드로 변환.

목적: 실제 로봇으로 뜬 맵 그대로의 환경을 시뮬에서 재현해, **월드 좌표계 = 맵 좌표계**로
      맞춘다. 그래서 nav2 의 map 좌표(예: 1.3924, -4.0099)가 시뮬에서도 동일 위치를 가리킴.

방법: 점유(검은) 픽셀을 greedy 사각형 병합으로 큰 박스 몇백 개로 줄여서 static 벽으로 emit.
      메시 파일 없이 순수 SDF box 라 gz 에서 경로 문제 없이 항상 로드됨.

좌표 변환 (맵 픽셀 → 월드 미터):
  픽셀 (col,row) 중심의 월드좌표 = (origin_x + (col+0.5)*res,  origin_y + (H-0.5-row)*res)
  (row 는 위에서부터라 y 를 뒤집음. origin/res 는 .yaml 에서.)

사용:
  python3 scripts/map_to_sdf.py \
      --map src/robot_bringup/maps/real_real_final_0604.yaml \
      --out src/ur3_4040_urdf/worlds/real_room.sdf
  (--height 벽높이 기본 1.0m, --wall-thresh 점유판정 픽셀값 기본 yaml occupied_thresh 사용)
"""
import argparse
import os

import numpy as np
import yaml
from PIL import Image


def load_occupancy(map_yaml):
    with open(map_yaml) as f:
        meta = yaml.safe_load(f)
    img_path = meta['image']
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(map_yaml), img_path)
    img = np.array(Image.open(img_path).convert('L'))  # H×W, 0..255
    res = float(meta['resolution'])
    origin = meta['origin']  # [x, y, yaw]
    occ_thresh = float(meta.get('occupied_thresh', 0.65))
    negate = int(meta.get('negate', 0))
    # 점유 확률 p = (255-pixel)/255 (negate=0). 점유 = p >= occ_thresh.
    p = img.astype(float) / 255.0
    if negate == 0:
        p = 1.0 - p
    occ = p >= occ_thresh
    return occ, res, origin


def merge_rectangles(occ):
    """occ(bool H×W) 를 겹치지 않는 사각형 목록 [(c0,r0,c1,r1), ...] 으로 greedy 분해."""
    H, W = occ.shape
    covered = np.zeros_like(occ, dtype=bool)
    rects = []
    for r in range(H):
        row_occ = occ[r]
        for c in range(W):
            if not row_occ[c] or covered[r, c]:
                continue
            # 가로로 최대한 확장
            c1 = c
            while c1 + 1 < W and occ[r, c1 + 1] and not covered[r, c1 + 1]:
                c1 += 1
            # 세로로 최대한 확장 (해당 열구간이 통째로 점유+미커버인 행만)
            r1 = r
            while (r1 + 1 < H and occ[r1 + 1, c:c1 + 1].all()
                   and not covered[r1 + 1, c:c1 + 1].any()):
                r1 += 1
            covered[r:r1 + 1, c:c1 + 1] = True
            rects.append((c, r, c1, r1))
    return rects


def rect_to_box(rect, res, origin, H):
    """사각형(픽셀) → (center_x, center_y, size_x, size_y) 월드 미터."""
    c0, r0, c1, r1 = rect
    ox, oy = origin[0], origin[1]
    cx = ox + ((c0 + c1) / 2.0 + 0.5) * res
    cy = oy + (H - 0.5 - (r0 + r1) / 2.0) * res
    sx = (c1 - c0 + 1) * res
    sy = (r1 - r0 + 1) * res
    return cx, cy, sx, sy


def build_sdf(rects, res, origin, H, height):
    walls = []
    for i, rect in enumerate(rects):
        cx, cy, sx, sy = rect_to_box(rect, res, origin, H)
        walls.append(f"""        <collision name="c{i}">
          <pose>{cx:.4f} {cy:.4f} {height/2:.4f} 0 0 0</pose>
          <geometry><box><size>{sx:.4f} {sy:.4f} {height:.4f}</size></box></geometry>
        </collision>
        <visual name="v{i}">
          <pose>{cx:.4f} {cy:.4f} {height/2:.4f} 0 0 0</pose>
          <geometry><box><size>{sx:.4f} {sy:.4f} {height:.4f}</size></box></geometry>
          <material><ambient>0.4 0.4 0.45 1</ambient><diffuse>0.5 0.5 0.55 1</diffuse></material>
        </visual>""")
    walls_xml = "\n".join(walls)
    return f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="real_room">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.2 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material>
        </visual>
      </link>
    </model>

    <model name="walls">
      <static>true</static>
      <link name="walls_link">
{walls_xml}
      </link>
    </model>
  </world>
</sdf>
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', required=True, help='맵 .yaml 경로')
    ap.add_argument('--out', required=True, help='출력 .sdf 경로')
    ap.add_argument('--height', type=float, default=1.0, help='벽 높이(m)')
    args = ap.parse_args()

    occ, res, origin = load_occupancy(args.map)
    H, W = occ.shape
    rects = merge_rectangles(occ)
    sdf = build_sdf(rects, res, origin, H, args.height)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, 'w') as f:
        f.write(sdf)
    print(f'맵 {W}x{H}px, 점유셀 {int(occ.sum())}개 → 벽 박스 {len(rects)}개')
    print(f'origin={origin}, res={res}m/px, 벽높이={args.height}m')
    print(f'→ {args.out} 저장 완료 (월드 좌표계 = 맵 좌표계)')


if __name__ == '__main__':
    main()
