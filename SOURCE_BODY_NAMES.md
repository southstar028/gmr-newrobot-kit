# Source human-body names (reference)

Each GMR motion source names the human skeleton differently. When you build a
`<source>_to_<robot>.json` config (and the `--human-tpose` for `scale_table.py`),
the human-body strings must match the source. The robot-side link map is the
same across sources; only these human names change.

The 14 key bodies, per source (as used in the G1 baseline configs):

| Role            | smplx (AMASS/OMOMO) | bvh_lafan1 (LAFAN1) | xrobot (VR/PICO) |
|-----------------|---------------------|----------------------|------------------|
| root            | `pelvis`            | `Hips`               | `Pelvis`         |
| torso/chest     | `spine3`            | `Spine2`             | `Spine3`         |
| left hip        | `left_hip`          | `LeftUpLeg`          | `Left_Hip`       |
| left knee       | `left_knee`         | `LeftLeg`            | `Left_Knee`      |
| left foot       | `left_foot`         | `LeftFootMod`        | `Left_Foot`      |
| right hip       | `right_hip`         | `RightUpLeg`         | `Right_Hip`      |
| right knee      | `right_knee`        | `RightLeg`           | `Right_Knee`     |
| right foot      | `right_foot`        | `RightFootMod`       | `Right_Foot`     |
| left shoulder   | `left_shoulder`     | `LeftArm`            | `Left_Shoulder`  |
| left elbow      | `left_elbow`        | `LeftForeArm`        | `Left_Elbow`     |
| left wrist      | `left_wrist`        | `LeftHand`           | `Left_Wrist`     |
| right shoulder  | `right_shoulder`    | `RightArm`           | `Right_Shoulder` |
| right elbow     | `right_elbow`       | `RightForeArm`       | `Right_Elbow`    |
| right wrist     | `right_wrist`       | `RightHand`          | `Right_Wrist`    |

Notes:
- These are the names observed in the GMR-shipped `*_to_g1.json` configs; if your
  GMR version differs, read the `ik_match_table1` keys of the relevant
  `<source>_to_g1.json` and use those.
- The robot link map (`{new_robot_link: g1_link}`) does **not** change with the
  source — it maps robot links only.
- For a new source family not listed here, just supply that source's
  `<source>_to_g1.json` as `--g1-config`; the tool reads its human names from it.
