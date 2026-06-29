# gmr-newrobot-kit

A helper toolkit to quickly add a **new humanoid robot** to
[GMR (General Motion Retargeting)](https://github.com/YanjieZe/GMR)
(paper: arXiv 2510.02252). It computes the two most tedious parts of writing an
`ik_config` — the **quat_offset** (axis alignment) and the **scale** (body-size
ratio) — in closed form, so adding a robot is mostly filling a template.

> This sits on top of GMR; it does not replace the GMR core.

> **New here? Start with [`GETTING_STARTED.md`](GETTING_STARTED.md)** — it lays
> out the end-to-end workflow (and is honest about the one manual step that is
> not automated).

## What it does
- **`quat_offset.py`** — computes each new-robot link's quat_offset. There is no
  single formula that is correct for every link, so it offers three modes
  (legs are easy; arms are not). **Read `METHOD.md`.** Validated against two
  in-house humanoids (Robot A, Robot B) with known-good configs: the
  `config-delta` mode reproduces their verified `xrobot_to_*` configs to
  **0.000–0.016°**, while a naive zero-pose FK delta is ~90° wrong on the arms.
  - `config-delta` (preferred) — transfer the per-link Δ from a source family
    where both G1 and the new robot already have a verified config.
  - `tpose` (bootstrap) — seed a brand-new robot from explicit robot+human
    T-poses (arms at ±90° so arm pose is captured).
  - `fk-delta` (legs only) — zero-pose FK Δ; correct for legs/torso, warns on arms.
- **`scale_table.py`** — computes an initial scale table from
  `scale = dist(robot_root, link) / dist(human_root, body)` (coordinate-frame
  independent).
- **`ik_config_template.json`** — annotated template with the 2-stage
  (table1 / table2) weight pattern pre-filled.
- **`CAUTIONS.md`** — ten pitfalls that actually cost people time.

## What you provide (inputs)
| Item | Description |
|---|---|
| New robot **MJCF** (.xml) | Loadable by MuJoCo. (URDF too, if available — for limit / DoF-order unification.) |
| New robot **rest/T-pose qpos** | `{root_pos, root_rot, degrees, joints:{...}}` json. The FK reference pose. |
| **Link map** | `{new_robot_link: g1_link}` for the 14 key bodies (pelvis / hip / knee / ankle / shoulder / elbow / wrist ×2 + torso). |
| **Source choice** | smplx / bvh_lafan1 / xrobot — run once per source (see "Multiple sources" below). Each has different human-body names. |
| (Ships with GMR) **G1 ik_config + G1 MJCF** | The baseline. Included in GMR. |
| **Human T-pose** | For scale: `{body: [x,y,z]}` keypoints matching the source. |

> Key point: with just the **robot MJCF + link-name map + source choice** you get
> quat_offset. Pose input is effectively unnecessary for the offset (zero-pose
> FK); a T-pose is only needed for scale.

## Quick start
```bash
pip install mujoco numpy scipy

# 1) quat_offset — pick the mode that fits your situation (see METHOD.md)

#   1a) BOOTSTRAP a brand-new robot (no verified config yet): seed from T-poses.
python -m newrobot_kit.quat_offset tpose \
    --robot-mjcf assets/my_robot/my_robot.xml \
    --robot-tpose my_robot_tpose.json \
    --human-tpose human_tpose.json \
    --ik-config smplx_to_my_robot.json \
    --table ik_match_table1 --out q1.json
#   ...then verify on video and fix per source.

#   1b) PREFERRED once one source is verified: transfer Δ to other sources.
python -m newrobot_kit.quat_offset config-delta \
    --src-g1 GMR/.../smplx_to_g1.json --src-new smplx_to_my_robot.json \
    --tgt-g1 GMR/.../xrobot_to_g1.json --link-map my_robot_link_map.json \
    --table ik_match_table1 --out xrobot_to_my_robot.quat.json

#   1c) legs-only quick check (warns on arms):
#   python -m newrobot_kit.quat_offset fk-delta --g1-config ... --g1-mjcf ... \
#       --new-mjcf ... --link-map ... --table ik_match_table1

# 2) initial scale table
python -m newrobot_kit.scale_table \
    --mjcf assets/my_robot/my_robot.xml \
    --robot-qpos my_robot_tpose.json \
    --human-tpose human_tpose.json \
    --ik-config smplx_to_my_robot.json

# 3) copy ik_config_template.json, fill the TODOs, paste the quat_offset/scale,
#    register the robot in GMR params.py, then run retargeting.
```
Example link map (`my_robot_link_map.json`):
```json
{ "my_l_ankle": "left_toe_link", "my_l_hip": "left_hip_yaw_link", "my_torso": "torso_link" }
```

## Multiple sources (smplx / bvh_lafan1 / xrobot)

GMR uses a **separate config per motion source**, because each source names its
human bodies differently and uses a different axis convention:

| Source | human_root | body naming (example) |
|---|---|---|
| **smplx** (AMASS/OMOMO) | `pelvis` | `left_hip`, `spine3`, `left_wrist` (lowercase) |
| **bvh_lafan1** (LAFAN1) | `Hips` | `LeftUpLeg`, `LeftLeg`, `Spine2`, `LeftHand` |
| **xrobot** (VR / PICO) | `Pelvis` | `Left_Hip`, `Spine3`, `Left_Wrist` (underscore) |

This kit is **source-agnostic by design**: the `--link-map` is robot↔G1 *robot*
links (source-independent), and the source-specific human-body names live in the
config you pass (per-source body-name table: `SOURCE_BODY_NAMES.md`). You build
**one config per source**. A typical order:

1. **Seed the first source** (e.g. smplx) with `tpose` mode, verify on video, fix.
2. **Transfer to the other sources** (bvh_lafan1, xrobot) with `config-delta`,
   using the verified smplx config as `--src-new`:
```bash
for tgt in bvh_lafan1 xrobot; do
  for tbl in ik_match_table1 ik_match_table2; do
    python -m newrobot_kit.quat_offset config-delta \
        --src-g1 GMR/.../smplx_to_g1.json --src-new smplx_to_my_robot.json \
        --tgt-g1 GMR/.../${tgt}_to_g1.json --link-map my_robot_link_map.json \
        --table "$tbl" --out "${tgt}_to_my_robot.${tbl}.json"
  done
done
```

Notes per source:
- **quat_offset** differs across sources because the source axis conventions
  differ — that is expected; build one config per source.
- **scale_table** `--human-tpose` keypoint names must match the source (use a
  smplx T-pose for the smplx config, etc.).
- **xrobot (VR)** suffers a world-frame flip; do not seed it directly with the
  T-pose-capture math — transfer to it with `config-delta` from a verified
  smplx/bvh config (`METHOD.md`, `CAUTIONS.md` §2).

## Method (why it is computed this way)
> Deeper theory — how the GMR paper's equations (Eq 2–6) map to the code and to this
> toolkit — is in [`THEORY.md`](THEORY.md).

- **quat_offset**: no single formula is right for every link — see `METHOD.md`.
  Legs/torso: zero-pose FK Δ from a G1 baseline is correct. Arms: it is ~90°
  wrong (G1's zero pose has arms raised), so arms come from an explicit T-pose
  (`tpose` mode) or from transferring a verified source config (`config-delta`).
  Validated to 0.000–0.016° against Robot A and Robot B.
- **scale**: a distance ratio, hence frame-independent. The root is scaled
  uniformly (avoids core-triangle distortion / foot sliding). Group the 14 raw
  values into ~2 tiers (legs / arms) instead of using them all.
- **2-stage weights**: stage 1 = end-effector position + all orientations,
  stage 2 = add light position on all bodies. Only feet + pelvis track position
  — this is the overfitting-avoidance design.

## Read this first
`CAUTIONS.md` — frame flip, overfitting (pos_offset≈0), MJCF/URDF limits, DoF
order, per-source configs, loss ≠ quality, and more.

## Credits & citation
This toolkit builds entirely on **GMR (General Motion Retargeting)** by
Yanjie Ze et al. — https://github.com/YanjieZe/GMR (MIT License). All credit for
the retargeting method, pipeline, and core code belongs to the GMR authors; this
kit only adds helper scripts for computing `quat_offset` / `scale` when onboarding
a new robot. GMR is **not** bundled here — obtain it separately.

If you use this kit, please cite the GMR work (BibTeX copied verbatim from the
[GMR repository](https://github.com/YanjieZe/GMR)):

```bibtex
@article{joao2025gmr,
  title={Retargeting Matters: General Motion Retargeting for Humanoid Motion Tracking},
  author= {Joao Pedro Araujo and Yanjie Ze and Pei Xu and Jiajun Wu and C. Karen Liu},
  year= {2025},
  journal= {arXiv preprint arXiv:2510.02252}
}

@article{ze2025twist,
  title={TWIST: Teleoperated Whole-Body Imitation System},
  author= {Yanjie Ze and Zixuan Chen and João Pedro Araújo and Zi-ang Cao and Xue Bin Peng and Jiajun Wu and C. Karen Liu},
  year= {2025},
  journal= {arXiv preprint arXiv:2505.02833}
}

@software{ze2025gmr,
  title={GMR: General Motion Retargeting},
  author= {Yanjie Ze and João Pedro Araújo and Jiajun Wu and C. Karen Liu},
  year= {2025},
  url= {https://github.com/YanjieZe/GMR},
  note= {GitHub repository}
}
```

## License / scope
This kit is released under the **MIT License** (see `LICENSE`), the same license
as GMR. It is an independent helper that runs on top of GMR. Robot assets,
SMPL-X body models, motion datasets, etc. follow their own sources' licenses.
This repo contains no proprietary robot assets, trained policies, or secrets.
