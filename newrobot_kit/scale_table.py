"""Compute an initial human_scale_table for a new robot (distance-ratio).

GMR scales the human skeleton per body in the root-local frame (paper Eq 2-3):

    scale[body] = dist(robot_root, robot_link) / dist(human_root, human_body)

This is a pure distance ratio, so it is coordinate-frame independent (unlike
quat_offset, scale is immune to frame flips). The root is scaled uniformly
(by the average of the leg scales) to avoid distorting the body-center
"triangle", which otherwise induces hip over-sway and foot sliding.

You normally do NOT use all 14 raw per-body values — group them into a small
number of tiers (e.g. legs vs arms, ~0.95 / ~0.85) to avoid per-joint
overfitting. See CAUTIONS.md.

Inputs you provide
------------------
- new robot MJCF + its rest/T-pose qpos (so FK gives link positions)
- a human T-pose: {body_name: [x, y, z]} keypoint positions
- the ik_config (for robot_root_name / human_root_name and the body mapping)

Usage
-----
    python -m newrobot_kit.scale_table \
        --mjcf path/to/<new_robot>.xml \
        --robot-qpos path/to/<new_robot>_tpose.json \
        --human-tpose path/to/human_tpose.json \
        --ik-config path/to/<src>_to_<new_robot>.json
"""
from __future__ import annotations

import argparse
import json

import numpy as np

try:
    import mujoco
except ImportError as e:  # pragma: no cover
    raise SystemExit(f"mujoco is required: pip install mujoco ({e})")


def robot_link_positions(mjcf_path: str, qpos_init: dict) -> dict[str, np.ndarray]:
    model = mujoco.MjModel.from_xml_path(mjcf_path)
    data = mujoco.MjData(model)
    data.qpos[:3] = qpos_init.get("root_pos", [0, 0, 0])
    if "root_rot" in qpos_init:
        data.qpos[3:7] = qpos_init["root_rot"]
    degrees = qpos_init.get("degrees", False)
    for jname, angle in qpos_init.get("joints", {}).items():
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
        if jid < 0:
            continue
        data.qpos[model.jnt_qposadr[jid]] = np.deg2rad(angle) if degrees else angle
    mujoco.mj_forward(model, data)
    return {model.body(i).name: data.body(model.body(i).name).xpos.copy()
            for i in range(model.nbody) if model.body(i).name}


def compute_scale_table(robot_pos, human_pos, ik_config) -> dict[str, float]:
    rr = ik_config["robot_root_name"]
    hr = ik_config["human_root_name"]
    robot_root, human_root = robot_pos[rr], human_pos[hr]
    table: dict[str, float] = {}
    for robot_link, entry in ik_config["ik_match_table1"].items():
        human_body = entry[0]
        if human_body == hr or robot_link not in robot_pos or human_body not in human_pos:
            continue
        rd = np.linalg.norm(robot_pos[robot_link] - robot_root)
        hd = np.linalg.norm(human_pos[human_body] - human_root)
        if hd < 1e-3:
            continue
        table[human_body] = round(float(rd / hd), 4)
        print(f"  {robot_link:24s} -> {human_body:16s}  robot={rd:.4f}  human={hd:.4f}  scale={rd/hd:.4f}")
    # root: uniform = mean of leg scales (Hip/Knee/Foot)
    legs = [v for k, v in table.items() if any(p in k for p in ("Hip", "Knee", "Foot", "hip", "knee", "foot"))]
    table[hr] = round(float(np.mean(legs)), 4) if legs else 1.0
    print(f"  root({hr}) = mean(leg scales) = {table[hr]}")
    return table


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mjcf", required=True)
    ap.add_argument("--robot-qpos", required=True, help="robot rest/T-pose qpos json")
    ap.add_argument("--human-tpose", required=True, help="{body: [x,y,z]} json")
    ap.add_argument("--ik-config", required=True)
    args = ap.parse_args()

    robot_pos = robot_link_positions(args.mjcf, json.load(open(args.robot_qpos)))
    human_raw = json.load(open(args.human_tpose))
    human_pos = {k: np.array(v[0] if isinstance(v[0], list) else v) for k, v in
                 (human_raw.get("body_data", human_raw)).items()}
    ik_config = json.load(open(args.ik_config))

    print("=== scale[body] = dist(robot_root, link) / dist(human_root, body) ===")
    table = compute_scale_table(robot_pos, human_pos, ik_config)
    print("\nRAW per-body scale (group into ~2 tiers before use — see CAUTIONS.md):")
    print(json.dumps(table, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
