# Cautions

Pitfalls that actually cost time when retargeting a new robot. Going through
these in order avoids most of the wasted effort.

## 1. Do NOT "optimize" quat_offset — it is a closed-form value
`quat_offset` is the **frame-convention difference** between a human body frame
and the corresponding robot link frame. It is not something to fit with gradient
descent. But there is **no single closed-form that is right for every link** —
legs are easy, arms are not (see `METHOD.md` and §1a below).

## 1a. Legs vs arms: pick the right mode
- **Legs/torso** — zero-pose FK Δ from a G1 baseline (`fk-delta` mode) is
  correct, because both robots stand at attention at zero pose.
- **Arms** — `fk-delta` is **~90° wrong**, because G1's zero pose has the arms
  raised (not at attention), so the pose difference leaks into the Δ. Get arms
  from an explicit robot T-pose (arms at ±90°, `tpose` mode) or by transferring a
  verified source config (`config-delta` mode). Validated: `fk-delta` matched the
  the known robot's legs to ≤0.02° but missed the arms by 91°; `config-delta`
  reproduced **all** links to ≤0.016°.

## 2. For VR (xrobot) sources — frame flip; transfer, don't capture
A T-pose-capture form `offset = human_pose.inv() * robot_pose` **breaks when the
human source frame and the MuJoCo world frame differ by a flip (mirror)**. The
flip leaks into the captured pose and the solver cannot separate "flip" from
"pose": scale diverges, feet tangle, left/right axes map mirror-wise. This is
common with VR (xrobot). So **do not seed xrobot directly** — build a verified
smplx/bvh config first, then `config-delta` onto xrobot (pose- and
flip-independent transfer).

## 3. Keep pos_offset at 0 — precise matching does not improve quality
Forcing every joint position to match the human (by tuning pos_offset/scale)
makes the result **less natural and introduces frame discontinuities**. The IK
is over-determined (more constraints than DoF), so least squares distributes the
error; aiming a constraint precisely at one joint amplifies compensating
rotations elsewhere (hip over-sway).
- Track **position only for the end-effectors (feet) and the pelvis**; let the
  rest follow orientation only (see the template weight pattern).
- Use pos_offset only when something is **visibly** wrong (self-collision,
  toed-in), and only a little.

## 4. Scale the root uniformly — do not distort the core triangle
Non-uniform precise scaling of pelvis–hip distorts the body-center triangle, so
the hip over-sways and the feet slide. Scale the root **uniformly** (mean of the
leg scales — `scale_table.py` does this). Do not paste the 14 raw values; group
them into ~2 tiers (e.g. legs 0.95 / arms 0.85) to avoid overfitting.

## 5. Unify MJCF ↔ URDF joint limits first
A common failure: retargeting (MJCF) passes, but the downstream trainer / real
robot (URDF) reports limit violations. Align the **joint limits and DoF order**
of both models before retargeting.

## 6. Check the DoF order
The DoF order of a GMR output pkl equals the MJCF actuator order. If it differs
from your training config or the real-robot SDK order, you need a reorder index.
Confirm with the `robot_body_names` / actuator list printed on load.

## 7. Keep one config per source family
Even for the same robot, smplx / bvh_lafan1 / xrobot differ in root name, body
names, axis convention, and scale. Do not assume that fixing one source's config
fixes another. Keep a separate `<source>_to_<robot>.json` per source.

## 8. Headless viewers
Some single-clip retargeting scripts have no viewer-off flag and crash headless
(GPU server). Use the dataset script (no viewer) or a headless variant.

## 9. NumPy version compatibility
A pkl produced under numpy 2 may fail to load under numpy 1.x (`numpy._core`).
Re-save the output pkl with the training environment's numpy.

## 10. Do not judge quality by loss
The IK loss (value function) is only a convergence signal. Judge retargeting
quality by **artifacts**: ground penetration, self-intersection, foot sliding,
waist/hip joint-value spikes, frame discontinuities. Verify by eye on video plus
these metrics.
