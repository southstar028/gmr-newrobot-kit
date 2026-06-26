# Method: computing quat_offset for a new robot

This document fixes the methodology **before** the code. It is grounded in a
validation run against two in-house humanoids (Robot A, Robot B) that already
have verified configs: the verified xrobot config is reproduced to **0.0–0.016°**
by the "config-to-config Δ" method, while a naive "zero-pose FK Δ" reproduces
legs exactly but is **~90° wrong on the arms**.

## What quat_offset is
In GMR, each IK target row carries a `quat_offset` (wxyz) applied as
`updated_quat = human_quat · quat_offset` (`motion_retarget.py`). It rotates a
human body frame into the corresponding robot link frame. It is a **fixed,
closed-form alignment value**, not something to optimize.

## The core difficulty: legs are easy, arms are not
The offset must capture the **link-frame convention difference** between human
body and robot link. Two ways to obtain that difference:

1. **Zero-pose FK Δ** — `Δ = R_g1_link(0)⁻¹ · R_new_link(0)` from forward
   kinematics at qpos = 0, then `offset_new = offset_g1 · Δ`.
   - Works **only if both robots' zero pose corresponds to the same human pose**.
   - **Legs/torso**: G1 and the new robot both stand at attention at zero pose →
     Δ is pure convention difference → **correct**.
   - **Arms**: G1's zero pose is *not* attention — its `shoulder_*` links carry a
     built-in rotation (lego/T-ish), while another robot's zero pose may be arms-
     down. The pose difference (~90°) leaks into Δ → **wrong arms**.
   - Verified: this is exactly what an early "g1baseline" backup attempt showed —
     legs ≤5.7°, arms 93°.

2. **Config-to-config Δ** — take a *source family* (smplx or bvh_lafan1) in which
   **both G1 and the new robot already have a verified config**, and transfer the
   per-link difference onto the target source (e.g. xrobot):
   ```
   Δ_link      = offset_<src>_g1[link]⁻¹ · offset_<src>_newrobot[link]
   offset_xrobot_newrobot[link] = offset_xrobot_g1[link] · Δ_link
   ```
   - Verified: reproduces the verified xrobot config of a known robot to ≤0.016° on **all** links,
     arms included (an in-house derive script).
   - Requires that a verified `<src>_to_newrobot.json` already exists.

## How the verified source configs themselves were made
Not one-shot automatic. The chain observed in the known-robot artifacts:

1. **T-pose-capture seed** (`compute_rot_offsets.py` + `pose_inits/<robot>_tpose.json`
   with shoulders at ±90°): `offset = human_tpose⁻¹ · robot_link_FK`. This
   produced the *arm* seed values (an early auto-generated config's arm quats match the
   final to a sign/component) — i.e. the explicit T-pose (arms raised 90°)
   injects the arm pose information that zero-pose FK lacks.
2. **G1-baseline + leg/torso correction**: legs/torso aligned via the G1 baseline
   (the "g1baseline" backup and other g1-baseline artifacts).
3. **Manual / per-source fix-ups** to a verified result. smplx and bvh arm offsets
   end up **90° apart** — each source is finished independently, not copied.

So: the **explicit robot T-pose (arms at ±90°) is what makes arms correct**;
zero-pose FK cannot, because the robot's *modeled* zero pose has arms down.

## Decision for this toolkit
Offer the right tool for each situation, and be honest about each one's validity:

- **`config-delta` mode (preferred when available)** — if a verified
  `<src>_to_newrobot.json` exists for at least one source, transfer Δ onto other
  sources (xrobot etc.). This is the method that reproduced the known robot to 0.016°.
- **`tpose` mode (bootstrap for a brand-new robot)** — compute
  `offset = human_tpose⁻¹ · robot_link_FK` from an explicit robot T-pose
  (shoulders at ±90°) and a human T-pose. This is how the first verified config
  is seeded; it gets arms approximately right because the T-pose carries arm
  pose. Then verify on video and fix per source.
- **`fk-delta` mode (legs only, with a loud warning)** — zero-pose FK Δ from a G1
  baseline. **Correct for legs/torso, wrong for arms** unless the new robot's
  zero pose is genuinely arms-at-attention. Never ship arms from this mode
  without checking.

**Do not present a single "one formula does everything" path.** The earlier
version of this toolkit did exactly that (fk-delta only) and was 90° wrong on
arms — caught only by validating against the two known robots.

## Sources
Run per source family (smplx / bvh_lafan1 / xrobot); each has different human
body names and a different axis convention, so offsets differ per source
(see `SOURCE_BODY_NAMES.md`). xrobot (VR) additionally suffers a world-frame flip
that the T-pose-capture method cannot separate — for xrobot, prefer config-delta
from a verified smplx/bvh result.
