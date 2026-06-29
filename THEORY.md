# Theory: the GMR paper, the code, and this toolkit

This document connects three layers so you can see **why** the toolkit configures a new robot
the way it does:

1. the **GMR paper** formulation — *General Motion Retargeting*, arXiv [`2510.02252`](https://arxiv.org/abs/2510.02252), Section IV (the 5-step pipeline, Eq 2–6);
2. the **GMR code** that implements it (`general_motion_retargeting/`, IK via [`mink`](https://github.com/kevinzakka/mink) — MuJoCo differential IK, `daqp` solver, damping `5e-1`, `max_iter 10`, convergence `< 1e-3`);
3. this **toolkit** — `quat_offset.py`, `scale_table.py`, and `ik_config_template.json` — which is exactly the per-robot configuration the paper's Steps 1–3 require.

> Scope: GMR covers the 5-step body retargeting. Neck/hand mapping and low-level tracking are
> separate systems (e.g. TWIST2, arXiv 2511.02832) and are not part of this toolkit.

---

## The 5-step pipeline (paper §IV ↔ code ↔ this toolkit)

| Step | Paper | GMR code | This toolkit |
|---|---|---|---|
| **1. Key-Body Matching** | mapping ℳ between human bodies and robot links + per-link position/orientation weights | each row of `ik_match_table1/2` = `[human_body, pos_w, rot_w, pos_offset, rot_offset]` | `ik_config_template.json` (the rows you fill in) · `SOURCE_BODY_NAMES.md` |
| **2. Rest-Pose Alignment** | a rotation offset (and optional local position offset) aligning each human body frame to the robot link frame | `offset_human_data()` — rotation first, position in the rotated frame | **`quat_offset.py`** (3 modes — see `METHOD.md`) |
| **3. Non-Uniform Local Scaling** | global height scale × per-body local scale; **root scaled uniformly** (Eq 2–3) | `scale_human_data()` + height `ratio` | **`scale_table.py`** |
| **4. Stage-1 IK** | end-effector **positions** + **all-body orientations** (Eq 4), solved by differential IK (Eq 5) | `mink.solve_ik(..., tasks1, ...)` | `ik_match_table1` weights |
| **5. Stage-2 Fine-Tuning** | warm-start from Stage-1, add **all key-body positions**, re-weighted w₂ (Eq 6) | `mink.solve_ik(..., tasks2, ...)` | `ik_match_table2` weights |

This toolkit is the configuration for Steps 1–3; Steps 4–5 are GMR's solver, driven by the
weights you set.

---

## Step 3 — scaling (Eq 2–3) ↔ `scale_table.py`

**Eq 2 (non-root body):**
```
p_target^b = (h/h_ref)·s^b·(p_source^j − p_source^root) + (h/h_ref)·s_root·p_source^root
```
**Eq 3 (root):**
```
p_target^root = (h/h_ref)·s_root·p_source^root
```
- `h/h_ref` — global height scale (source key length ÷ reference key length).
- `s^b` — per-body local scale; `s_root` — root scale, applied **uniformly**.
- The paper scales the root uniformly **to avoid foot-sliding artifacts**.

In code, the height ratio multiplies the whole table, and per-body scaling is applied in
root-relative coordinates — i.e. the `(p_j − p_root)·s^b` term of Eq 2:

```python
ratio = actual_human_height / ik_config["human_height_assumption"]
human_scale_table[key] *= ratio
human_data_local[body] = (human_data[body][0] - root_pos) * human_scale_table[body]
```

`scale_table.py` builds these per-body factors from limb-length ratios (e.g. legs ≈ 0.95,
arms ≈ 0.85 for one humanoid) with the root kept uniform — see `METHOD.md` for how the numbers
are chosen and **why over-precise per-body scaling distorts the core triangle**.

---

## Step 2 — rest-pose alignment ↔ `quat_offset.py`

GMR applies the offset as `updated_quat = human_quat · quat_offset`, rotation first, then the
position offset **in the rotated frame**:

```python
updated_quat      = human_quat * rot_offset                  # orientation alignment
global_pos_offset = R(updated_quat).apply(local_pos_offset)  # position offset in the rotated frame
pos               = pos + global_pos_offset
```

So `quat_offset` is a **fixed, closed-form alignment** between a human body frame and a robot
link frame — not something to optimize — and changing `rot_offset` means `pos_offset` must be
recomputed (it lives in the rotated frame).

The hard part is that **legs are easy and arms are not**: a zero-pose FK difference is correct
for legs/torso but ~90° wrong on arms (the modeled zero pose has arms down, not at attention).
`quat_offset.py` therefore offers three honest modes — `config-delta` (preferred), `tpose`
(bootstrap), `fk-delta` (legs only, with a warning). The full reasoning, with validation
against two known robots (reproduced to ≤ 0.016°), is in **`METHOD.md`**.

---

## Steps 4–5 — two-stage IK (Eq 4 / 5 / 6)

**Stage-1 (Eq 4)** — end-effector positions + all-body orientations:
```
min_q  Σ_ℳ    w1^R_ij · ‖ R_i^h ⊖ R_j(q) ‖²          (all body orientations)
     + Σ_ℳ_ee w1^p_ij · ‖ p_i^target − p_j(q) ‖²       (end-effectors only)
  s.t.  q⁻ ≤ q ≤ q⁺
```
`R_i^h ⊖ R_j(q)` is the **geodesic** SO(3) difference (log map), not a Frobenius norm. `ℳ_ee`
is the end-effectors (hands/feet) — feet get position, everything else only orientation.

**Differential IK (Eq 5)** — what `mink` actually solves each step:
```
min_q̇  ‖ e(q) + J(q)·q̇ ‖²_W      s.t.  q⁻ ≤ q + q̇·Δt ≤ q⁺
```
It solves for joint **velocities** and integrates them, rather than for angles directly.

**Stage-2 (Eq 6)** — warm-started from Stage-1, now including all key-body positions, re-weighted:
```
min_q  Σ w2^R_ij·‖R_i^h ⊖ R_j(q)‖²  +  w2^p_ij·‖p_i^target − p_j(q)‖²
```

The Eq 4 → Eq 6 transition is **visible in the weights** you set. Example pattern:

| link | `ik_match_table1` (pos / rot) ≈ Eq 4 | `ik_match_table2` (pos / rot) ≈ Eq 6 |
|---|---|---|
| pelvis (root) | 100 / 10 | 100 / 5 |
| ankle (end-effector) | 100 / 0 | 100 / 0 |
| hip · knee (intermediate) | **0 / 10** (orientation only) | **10 / 5** (position added) |

Stage-1 pins the feet and all orientations; Stage-2 *adds* intermediate-joint positions.

---

## Why these choices (design rationale)

1. **Two stages = local-minimum avoidance.** Fix the kinematically decisive constraints first
   (orientations + foot positions), then refine the rest from that solution. The
   `table1 → table2` order is this idea.
2. **Root uniform scaling (Eq 3) preserves the core triangle.** Precisely non-uniform pelvis/hip
   scaling distorts the body-center triangle, causing hip oscillation and foot sliding — the
   same reason the paper scales the root uniformly.
3. **Position only on end-effectors (`pos_offset ≈ 0`).** Forcing precise position matches on
   intermediate joints over-determines the system and amplifies compensatory rotation. Let
   orientation carry the pose; pin position only where it matters (feet, pelvis).
4. **Loss ≠ quality.** The IK value function is a convergence signal, not a quality metric.
   Quality is judged by artifacts (ground penetration, self-intersection, velocity spikes),
   per the paper's §V-B.

---

## File map

| file | role | paper step |
|---|---|---|
| `ik_config_template.json` | mapping ℳ + per-link weights + offsets/scale | 1 · 2 · 3 |
| `quat_offset.py` | rest-pose rotation offsets (3 modes) | 2 |
| `scale_table.py` | per-body + root scale factors (Eq 2–3) | 3 |
| `general_motion_retargeting/motion_retarget.py` (GMR) | applies scale/offset + two-stage IK | 3 · 4 · 5 |
| `general_motion_retargeting/kinematics_model.py` (GMR) | robot FK — `p_j(q)`, `R_j(q)` | 4 · 6 |

See also: `METHOD.md` (quat_offset derivation), `SOURCE_BODY_NAMES.md` (per-source body names),
`GETTING_STARTED.md` (end-to-end), `CAUTIONS.md` (pitfalls).

---

## Citation

If you use GMR, cite the paper (BibTeX in the repo README); IK is built on `mink`.
This toolkit only adds the per-robot configuration around GMR's solver.
