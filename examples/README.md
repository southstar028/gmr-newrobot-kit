# Example: adding a new robot

A minimal walkthrough. Replace `my_robot` with your robot name.

## 0. Prerequisites
- GMR installed and importable, with the Unitree G1 assets/configs available.
- Your robot MJCF at `assets/my_robot/my_robot.xml` (MuJoCo-loadable).

## 1. Define the link map
Map your robot's 14 key links to the kinematically equivalent G1 links. Use the
body names printed when MuJoCo loads each MJCF.

`my_robot_link_map.json`:
```json
{
  "my_pelvis":     "pelvis",
  "my_l_hip":      "left_hip_yaw_link",
  "my_l_knee":     "left_knee_link",
  "my_l_ankle":    "left_toe_link",
  "my_r_hip":      "right_hip_yaw_link",
  "my_r_knee":     "right_knee_link",
  "my_r_ankle":    "right_toe_link",
  "my_torso":      "torso_link",
  "my_l_shoulder": "left_shoulder_yaw_link",
  "my_l_elbow":    "left_elbow_link",
  "my_l_wrist":    "left_wrist_yaw_link",
  "my_r_shoulder": "right_shoulder_yaw_link",
  "my_r_elbow":    "right_elbow_link",
  "my_r_wrist":    "right_wrist_yaw_link"
}
```
Note the deliberate choices: ankle ↔ G1 `toe` (both are the foot end-effector),
hip_pitch ↔ G1 `hip_yaw` (both are the topmost leg link). Match by **kinematic
role**, not by name.

## 2. Compute quat_offset (read METHOD.md for mode choice)

**Brand-new robot, no verified config yet → seed with `tpose`** (needs an
explicit robot T-pose with arms at ±90° and a matching human T-pose):
```bash
python -m newrobot_kit.quat_offset tpose \
    --robot-mjcf assets/my_robot/my_robot.xml \
    --robot-tpose my_robot_tpose.json \
    --human-tpose human_tpose_smplx.json \
    --ik-config smplx_to_my_robot.json \
    --table ik_match_table1 --out q1.json
# repeat for ik_match_table2. Then verify on video and fix the smplx config.
```

**Once smplx is verified → transfer to bvh_lafan1 / xrobot with `config-delta`**
(this is the validated path — reproduced two known robots' configs to ≤0.016°):
```bash
python -m newrobot_kit.quat_offset config-delta \
    --src-g1 GMR/.../smplx_to_g1.json --src-new smplx_to_my_robot.json \
    --tgt-g1 GMR/.../xrobot_to_g1.json --link-map my_robot_link_map.json \
    --table ik_match_table1 --out xrobot_to_my_robot.t1.json
```

**Quick legs-only sanity check** (`fk-delta`) prints a per-link Δ and warns on
arms — useful to confirm leg conventions, not for shipping arm values.

## 3. Compute the initial scale table
```bash
python -m newrobot_kit.scale_table \
    --mjcf assets/my_robot/my_robot.xml \
    --robot-qpos my_robot_tpose.json \
    --human-tpose human_tpose.json \
    --ik-config smplx_to_my_robot.json
```
Group the raw output into two tiers (legs vs arms) before pasting.

## 4. Assemble the config
Copy `../newrobot_kit/ik_config_template.json` to
`smplx_to_my_robot.json`, replace the `TODO_*` link names, paste the
quat_offset values (field index 4) and the grouped scale table.

## 5. Register in GMR and run
Add the robot to GMR `params.py` (ROBOT_XML_DICT / IK_CONFIG_DICT /
ROBOT_BASE_DICT / viewer cam), then run a single-clip retarget and inspect on
video. Iterate using `CAUTIONS.md` if you see artifacts.
