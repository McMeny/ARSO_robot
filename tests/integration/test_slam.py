"""SLAM Toolbox mapping pass in sim: the robot wanders (Nav2 on the live SLAM map), the map is saved with
map_saver_cli and compared with the committed prebuilt map (ground-truth rasterization of the world).
This is the same procedure DEPLOY.md uses to map the demo room."""
from __future__ import annotations

import time
from pathlib import Path

import numpy as np
import pytest
import yaml
from scipy.ndimage import distance_transform_edt

from harness import ROOT, bash, bash_run, linux_root, ros_available, CLEAN

pytestmark = pytest.mark.skipif(not ros_available(), reason="ROS 2 workspace not available")


def read_pgm(stem: Path):
    meta = yaml.safe_load(stem.with_suffix(".yaml").read_text())
    raw = stem.with_suffix(".pgm").read_bytes()
    tokens, i = [], 0
    while len(tokens) < 4:                     # magic, width, height, maxval (comments allowed between)
        while raw[i:i + 1].isspace():
            i += 1
        if raw[i:i + 1] == b"#":
            i = raw.index(b"\n", i) + 1
            continue
        j = i
        while not raw[j:j + 1].isspace():
            j += 1
        tokens.append(raw[i:j])
        i = j
    w, h = int(tokens[1]), int(tokens[2])
    img = np.frombuffer(raw[i + 1:i + 1 + w * h], np.uint8).reshape(h, w)[::-1]
    return img, meta


def occupied_world_points(img, meta):
    r, c = np.nonzero(img < 50)
    res, (ox, oy, _) = meta["resolution"], meta["origin"]
    return np.stack([ox + (c + 0.5) * res, oy + (r + 0.5) * res], 1)


def test_slam_mapping_matches_world():
    bash_run(CLEAN, 30)
    log = open(ROOT / "outputs/logs/slam_field.log", "w")
    proc = bash("ros2 launch eyesteer_bringup slam.launch.py sim:=true world:=field robot:=field_robot", stdout=log, stderr=log)
    try:
        time.sleep(150)          # wander + map
        out_stem = "/tmp/eyesteer_slam_field"
        print(bash_run(f"ros2 run nav2_map_server map_saver_cli -f {out_stem} --ros-args -p use_sim_time:=true", 60))
        win = ROOT / "outputs/logs/slam_field"
        bash_run(f"cp {out_stem}.pgm {linux_root()}/outputs/logs/slam_field.pgm && cp {out_stem}.yaml {linux_root()}/outputs/logs/slam_field.yaml", 20)
    finally:
        bash_run(CLEAN, 30)
        proc.kill()
        log.close()
    slam_img, slam_meta = read_pgm(win)
    ref_img, ref_meta = read_pgm(ROOT / "ros2_ws/src/eyesteer_bringup/maps/field")
    s_pts = occupied_world_points(slam_img, slam_meta)
    assert len(s_pts) > 300, "SLAM map nearly empty"
    # distance from each SLAM-occupied cell to the nearest ground-truth occupied cell
    ref_occ = ref_img < 50
    dist = distance_transform_edt(~ref_occ) * ref_meta["resolution"]
    ox, oy = ref_meta["origin"][:2]
    c = ((s_pts[:, 0] - ox) / ref_meta["resolution"]).astype(int).clip(0, ref_img.shape[1] - 1)
    r = ((s_pts[:, 1] - oy) / ref_meta["resolution"]).astype(int).clip(0, ref_img.shape[0] - 1)
    precision = float(np.mean(dist[r, c] <= 0.15))
    print(f"SLAM precision (occupied cells within 15 cm of the true walls/obstacles): {precision:.3f}, {len(s_pts)} cells")
    assert precision > 0.85
