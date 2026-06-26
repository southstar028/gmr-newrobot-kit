# Getting started

How a new user actually uses this kit. Start here.

## First, the honest scope
This kit does **not** fully automate adding a new robot. There is a manual
verification step you cannot skip. What it *does* remove is the two parts that
are pure math (quat_offset, scale) and the repetitive work of redoing the arms
for every motion source. Concretely:

- **It gives you a good starting config** for a brand-new robot (`tpose` mode),
  including approximate arms — much better than hand-guessing.
- **Once one source is verified, it propagates to the others exactly**
  (`config-delta`, validated to ≤0.016°), so you don't re-solve arms per source.
- **You still verify on video and fix the first source by hand.** That iteration
  is the real work and is not automated.

## Which path are you on?

```
Do you already have a verified <source>_to_<your_robot>.json for ANY source?
        │
        ├── NO  → Path A (brand-new robot): bootstrap, then expand.
        └── YES → Path B: just expand to the other sources.
```

## Path A — brand-new robot (no config yet)

**Inputs to prepare**
| Input | What |
|---|---|
| Robot MJCF (`.xml`) | loadable by MuJoCo |
| Robot T-pose qpos (`*_tpose.json`) | `{root_pos, root_rot, degrees, joints:{...}}`, **arms at ±90°** |
| Human T-pose | `{body:[x,y,z]}` for the source (e.g. smplx); names per `SOURCE_BODY_NAMES.md` |
| Link map (`link_map.json`) | `{your_robot_link: g1_link}` for the 14 key bodies, matched by kinematic role |
| GMR's G1 config + MJCF | ships with GMR (baseline) |

**Steps**
1. **Seed your first source** (pick one, e.g. smplx) — gets legs right and arms
   approximately right:
   ```bash
   python -m newrobot_kit.quat_offset tpose \
       --robot-mjcf my_robot.xml --robot-tpose my_robot_tpose.json \
       --human-tpose human_tpose_smplx.json --ik-config smplx_to_my_robot.json \
       --table ik_match_table1   # repeat for ik_match_table2
   ```
2. **Scale**:
   ```bash
   python -m newrobot_kit.scale_table --mjcf my_robot.xml \
       --robot-qpos my_robot_tpose.json --human-tpose human_tpose_smplx.json \
       --ik-config smplx_to_my_robot.json
   ```
   Group the printed per-body values into ~2 tiers (legs / arms).
3. **Assemble** `smplx_to_my_robot.json` from `newrobot_kit/ik_config_template.json`
   (replace `TODO_*` links, paste quat_offset into field [4], paste grouped scale).
4. **Register in GMR** `params.py` and run a single clip.
5. **★ Verify on video and fix** (the manual step). Use `CAUTIONS.md`. Iterate on
   the offending links until this one source looks right. Now smplx is *verified*.
6. **Expand to the other sources** → go to Path B with smplx as the verified source.

## Path B — expand to other sources (validated, accurate)

You have one verified source (say `smplx_to_my_robot.json`). Transfer it:
```bash
for tgt in bvh_lafan1 xrobot; do
  for tbl in ik_match_table1 ik_match_table2; do
    python -m newrobot_kit.quat_offset config-delta \
        --src-g1 GMR/.../smplx_to_g1.json --src-new smplx_to_my_robot.json \
        --tgt-g1 GMR/.../${tgt}_to_g1.json --link-map link_map.json \
        --table "$tbl" --out "${tgt}_to_my_robot.${tbl}.json"
  done
done
```
Paste the results into `bvh_lafan1_to_my_robot.json` / `xrobot_to_my_robot.json`
(field [4]), set each one's scale table, register, and run. This step reproduced
two known robots' verified configs to ≤0.016°, so the other sources should need
little to no hand-tuning — but still glance at the video.

> **xrobot (VR)**: always reach it via `config-delta`, never seed it directly —
> its world-frame flip breaks the T-pose-capture math (`CAUTIONS.md` §2).

## What success looks like
Per source, on a test clip: feet planted (no sliding), root height sensible, no
self-penetration, no waist/hip spikes, motion continuous. Judge by these
artifacts, **not** by IK loss (`CAUTIONS.md` §10).

## When the arms are still wrong
The arms are the hard part. If `tpose` arms look off after step 5:
- Re-check the robot T-pose actually has arms at ±90° (not arms-down).
- Confirm the link map matches by **kinematic role** (shoulder↔shoulder, etc.).
- See `METHOD.md` "legs are easy, arms are not" and `CAUTIONS.md` §1a.
