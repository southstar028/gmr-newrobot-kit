"""Compute GMR ik_config quat_offsets for a NEW robot.

Read METHOD.md first. There is NO single formula that gets everything right.
quat_offset aligns a human body frame to a robot link frame
(`updated_quat = human_quat . quat_offset` in motion_retarget.py). It is a
fixed closed-form value, not something to optimize.

This module offers three modes, validated against two in-house humanoids
(Robot A, Robot B) with known-good configs:

  config-delta  (preferred)  transfer Δ from a source family where BOTH g1 and
                             the new robot already have a verified config, onto
                             another source (e.g. xrobot). Reproduces the
                             verified config of a known robot to <=0.016 deg on ALL links.

  tpose         (bootstrap)  offset = human_tpose^-1 . robot_link_FK, using an
                             EXPLICIT robot T-pose (shoulders at +-90 deg) and a
                             human T-pose. Gets arms approximately right because
                             the T-pose carries arm pose. Use to seed the FIRST
                             verified config, then verify and fix per source.

  fk-delta      (legs only)  Δ = R_g1_link(0)^-1 . R_new_link(0) from zero-pose
                             FK, offset_new = offset_g1 . Δ. CORRECT for
                             legs/torso, ~90 deg WRONG for arms unless the new
                             robot's zero pose is genuinely arms-at-attention.
                             Prints a warning on arm links.

Usage
-----
  # preferred: you already have <src>_to_<newrobot>.json for one source
  python -m newrobot_kit.quat_offset config-delta \
      --src-g1   smplx_to_g1.json        --src-new   smplx_to_myrobot.json \
      --tgt-g1   xrobot_to_g1.json        --tgt-new-out xrobot_to_myrobot.quat.json \
      --link-map link_map.json --table ik_match_table1

  # bootstrap a brand-new robot from explicit T-poses
  python -m newrobot_kit.quat_offset tpose \
      --robot-mjcf myrobot.xml --robot-tpose myrobot_tpose.json \
      --human-tpose human_tpose.json --ik-config smplx_to_myrobot.json \
      --table ik_match_table1

  # legs-only FK delta (warns on arms)
  python -m newrobot_kit.quat_offset fk-delta \
      --g1-config smplx_to_g1.json --g1-mjcf g1.xml \
      --new-mjcf myrobot.xml --link-map link_map.json --table ik_match_table1
"""
from __future__ import annotations

import argparse
import json

import numpy as np
from scipy.spatial.transform import Rotation as R

ARM_HINTS = ("shoulder", "elbow", "wrist", "arm", "hand")


def _is_arm(link: str) -> bool:
    l = link.lower()
    return any(h in l for h in ARM_HINTS)


def _load(path: str) -> dict:
    return json.load(open(path))


def _offsets(cfg: dict, table: str) -> dict[str, R]:
    return {link: R.from_quat(e[4], scalar_first=True) for link, e in cfg[table].items()}


def _geodesic_deg(a: R, b: R) -> float:
    return float(np.degrees((a.inv() * b).magnitude()))


# ---------------------------------------------------------------- config-delta
def config_delta(src_g1, src_new, tgt_g1, link_map, table):
    """offset_tgt_new[link] = offset_tgt_g1[g1link] . (offset_src_g1[g1link]^-1 . offset_src_new[newlink]).

    link_map: {new_robot_link: g1_link}. src_* and tgt_* are loaded configs.
    Returns {new_robot_link: [w,x,y,z]}.
    """
    o_sg, o_sn = _offsets(src_g1, table), _offsets(src_new, table)
    o_tg = _offsets(tgt_g1, table)
    out, missing = {}, []
    for new_link, g1_link in link_map.items():
        if g1_link not in o_sg or new_link not in o_sn or g1_link not in o_tg:
            missing.append((new_link, g1_link))
            continue
        delta = o_sg[g1_link].inv() * o_sn[new_link]      # source's verified g1->new diff
        offset_new = o_tg[g1_link] * delta                # transfer onto target source's g1 offset
        out[new_link] = [round(float(x), 6) for x in offset_new.as_quat(scalar_first=True)]
        print(f"  {new_link:26s} <- g1:{g1_link:24s}  Δ={np.degrees(delta.magnitude()):6.2f}deg")
    for nl, gl in missing:
        print(f"  [skip] {nl} <- {gl} (missing in a config)")
    return out


# ---------------------------------------------------------------------- tpose
def tpose(robot_mjcf, robot_tpose, human_tpose, ik_config, table):
    """offset[link] = R_human_body(T-pose)^-1 . R_robot_link_FK(T-pose).

    Seeds a config for a brand-new robot. Robot T-pose should have arms at +-90
    so arm pose is represented. ik_config gives the link->human_body mapping.
    """
    import mujoco  # local import: only this mode needs it
    model = mujoco.MjModel.from_xml_path(robot_mjcf)
    data = mujoco.MjData(model)
    qi = _load(robot_tpose)
    data.qpos[:3] = qi.get("root_pos", [0, 0, 0])
    if "root_rot" in qi:
        data.qpos[3:7] = qi["root_rot"]
    deg = qi.get("degrees", False)
    for j, a in qi.get("joints", {}).items():
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        if jid >= 0:
            data.qpos[model.jnt_qposadr[jid]] = np.deg2rad(a) if deg else a
    mujoco.mj_forward(model, data)

    human = _load(human_tpose)
    human = human.get("body_data", human)
    cfg = _load(ik_config)
    out, missing = {}, []
    for link, entry in cfg[table].items():
        hb = entry[0]
        if hb not in human:
            missing.append((link, hb)); continue
        try:
            R_robot = R.from_matrix(data.body(link).xmat.reshape(3, 3))
        except Exception:
            missing.append((link, hb)); continue
        hq = human[hb][1] if isinstance(human[hb][0], (list, tuple)) else human[hb]
        R_human = R.from_quat(np.asarray(hq, float), scalar_first=True)
        offset = R_human.inv() * R_robot
        out[link] = [round(float(x), 6) for x in offset.as_quat(scalar_first=True)]
        print(f"  {link:26s} <- {hb}")
    for l, hb in missing:
        print(f"  [skip] {l} <- {hb} (missing in MJCF or human T-pose)")
    return out


# ------------------------------------------------------------------- fk-delta
def fk_delta(g1_config, g1_mjcf, new_mjcf, link_map, table):
    """offset_new[link] = offset_g1[g1link] . (R_g1_link(0)^-1 . R_new_link(0)).

    Zero-pose FK delta. Correct for legs/torso, WRONG for arms unless the new
    robot's zero pose is arms-at-attention. Warns on arm links.
    """
    import mujoco

    def world_rots(path):
        m = mujoco.MjModel.from_xml_path(path)
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)
        return {m.body(i).name: R.from_matrix(d.body(m.body(i).name).xmat.reshape(3, 3))
                for i in range(m.nbody) if m.body(i).name}

    Rg1, Rnew = world_rots(g1_mjcf), world_rots(new_mjcf)
    o_g1 = _offsets(g1_config, table)
    out, missing, arm_warned = {}, [], False
    for new_link, g1_link in link_map.items():
        if g1_link not in Rg1 or new_link not in Rnew or g1_link not in o_g1:
            missing.append((new_link, g1_link)); continue
        delta = Rg1[g1_link].inv() * Rnew[new_link]
        offset_new = o_g1[g1_link] * delta
        out[new_link] = [round(float(x), 6) for x in offset_new.as_quat(scalar_first=True)]
        warn = "  <-- ARM: verify, likely wrong" if _is_arm(new_link) else ""
        if warn:
            arm_warned = True
        print(f"  {new_link:26s} <- g1:{g1_link:24s}  Δ={np.degrees(delta.magnitude()):6.2f}deg{warn}")
    for nl, gl in missing:
        print(f"  [skip] {nl} <- {gl}")
    if arm_warned:
        print("\n  [WARNING] fk-delta arm offsets are unreliable (G1 zero pose has arms"
              " raised, not at attention). Use config-delta or tpose for arms. See METHOD.md.")
    return out


# ----------------------------------------------------------------------- main
def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="mode", required=True)

    a = sub.add_parser("config-delta", help="transfer Δ from a verified source config (preferred)")
    a.add_argument("--src-g1", required=True);  a.add_argument("--src-new", required=True)
    a.add_argument("--tgt-g1", required=True);  a.add_argument("--link-map", required=True)
    a.add_argument("--table", default="ik_match_table1")
    a.add_argument("--out", dest="tgt_new_out", default=None)

    b = sub.add_parser("tpose", help="seed from explicit robot+human T-poses (bootstrap)")
    b.add_argument("--robot-mjcf", required=True); b.add_argument("--robot-tpose", required=True)
    b.add_argument("--human-tpose", required=True); b.add_argument("--ik-config", required=True)
    b.add_argument("--table", default="ik_match_table1"); b.add_argument("--out", default=None)

    c = sub.add_parser("fk-delta", help="zero-pose FK Δ (legs only; warns on arms)")
    c.add_argument("--g1-config", required=True); c.add_argument("--g1-mjcf", required=True)
    c.add_argument("--new-mjcf", required=True);  c.add_argument("--link-map", required=True)
    c.add_argument("--table", default="ik_match_table1"); c.add_argument("--out", default=None)

    args = ap.parse_args()

    if args.mode == "config-delta":
        print(f"=== config-delta [{args.table}] (preferred; reproduces verified configs) ===")
        res = config_delta(_load(args.src_g1), _load(args.src_new), _load(args.tgt_g1),
                           _load(args.link_map), args.table)
        out_path = args.tgt_new_out
    elif args.mode == "tpose":
        print(f"=== tpose seed [{args.table}] (arms approx from explicit T-pose) ===")
        res = tpose(args.robot_mjcf, args.robot_tpose, args.human_tpose, args.ik_config, args.table)
        out_path = args.out
    else:
        print(f"=== fk-delta [{args.table}] (legs OK, arms unreliable) ===")
        res = fk_delta(_load(args.g1_config), args.g1_mjcf, args.new_mjcf,
                       _load(args.link_map), args.table)
        out_path = args.out

    print("\nquat_offset (wxyz) — paste into ik_match_table field [4]:")
    print(json.dumps(res, indent=2, ensure_ascii=False))
    if out_path:
        json.dump(res, open(out_path, "w"), indent=2, ensure_ascii=False)
        print(f"\n[OK] wrote {out_path}")


if __name__ == "__main__":
    main()
