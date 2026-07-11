# NEAR-field visual servo — pre-2026-07-07 version (legacy — superseded)

> ⚠️ **This document describes the Phase 5 NEAR-field corrector as it shipped before the
> 2026-07-07 rewrite.** It is **kept for historical context and rollback reference** — it is
> **not** the code running today, and nothing in `dock_trigger.py` reads this document.
>
> The FAR/NEAR hand-over structure (`freeze_axis_distance`) and the overall 7-phase sequencer
> are unchanged — only the NEAR-field steering law described here was rewritten. For the
> current implementation, see:
> - [`13_perception_and_line.md`](13_perception_and_line.md) — perpendicular-line tracking and
>   the FAR-regime axis estimate (unaffected by this change)
> - [`05_parameters.md`](05_parameters.md) — current parameter meanings
> - the in-code comments in `_final_visual_approach()` in
>   [`scripts/dock_trigger.py`](../scripts/dock_trigger.py)
>
> **Exact rollback point:** the git tag `docking-legacy-pre-2026-07-07-near-approach` marks the
> last commit before this rewrite started. To see the file as it was:
> `git show docking-legacy-pre-2026-07-07-near-approach:ros2/src/openamrobot_docking/scripts/dock_trigger.py`.
> To fully revert the NEAR corrector while keeping everything merged after it, `git revert` the
> `feature/docking-near-approach` PR merge commit rather than hand-editing — the tag is there so
> the exact starting point is never ambiguous even if that PR's branch is later deleted.

---

## Why this document exists

The night of 2026-07-07, real-robot testing of the final approach (NEAR regime, camera→tag
depth ≤ `freeze_axis_distance`, ~0.25–0.70 m) showed a large oscillation with poor reactivity in
the last seconds before contact. Three real bugs were found and fixed (below). The fixes were
validated on hardware ("c'est aligné") but only over a single evening with a battery that ran low
by the end — this is a good rewrite, not a battle-tested one. This document lets a future session
compare against the exact prior behaviour, or fall back to it wholesale, without having to dig
through git log.

---

## The old NEAR steering law

```python
raw_angle = atan2(c1cam[0], c1cam[2])      # bearing to the centre tag, camera_optical_frame

if filtered_angle is None:
    filtered_angle = raw_angle
    d_angle = 0.0
elif abs(raw_angle - filtered_angle) > visual_servo_max_step:
    d_angle = 0.0                          # solvePnP flicker — reject, coast on last
else:
    prev_angle = filtered_angle
    filtered_angle = alpha * raw_angle + (1 - alpha) * filtered_angle
    d_angle = (filtered_angle - prev_angle) / period     # <-- FIXED period, not real dt

# hysteresis deadband (unchanged by the rewrite, still in the current code)
if abs(filtered_angle) > deadband: correcting = True
elif abs(filtered_angle) < 0.4 * deadband: correcting = False

omega = -(kp * filtered_angle + kd * d_angle) if correcting else 0.0
```

Depth (`c1cam[2]`) was read only to decide the FAR→NEAR hand-over; the steering law itself never
used it — a bearing-only PD controller on `atan2(X, Z)`.

## The three bugs this had

1. **Fixed-`period` derivative.** `d_angle` divided by the nominal control period (0.05 s)
   instead of the real elapsed time since the last *valid* sample. AprilTag detections were
   observed missing ~25% of frames in NEAR; every time a tag reappeared after a multi-frame gap,
   the angle delta got divided by a `period` far smaller than the real elapsed time, inflating the
   derivative term into a sharp corrective jerk exactly when the signal was staler, not fresher.

2. **No depth compensation.** `atan2(X, Z)` mechanically grows for the *same physical lateral
   error* as `Z` (depth) shrinks — the closer the robot gets, the more aggressively the same
   1 cm of true offset reads as an angular error, so `kp` that was calm at 0.6 m became jumpy at
   0.25 m without the robot's real position error changing.

3. **No lookahead, no lateral-offset reasoning.** Because the law steered on `atan2(X, Z)`
   directly rather than reasoning about a lateral offset against a fixed lookahead distance (as
   the FAR regime already did with `line_lookahead_distance`), the NEAR and FAR regimes had
   qualitatively different response curves — the hand-over between them was not smooth in gain
   terms even though the position was continuous.

## What replaced it (current code, 2026-07-07 onward)

- Real elapsed time (`now - last_valid_t`) for the derivative, with the derivative explicitly
  zeroed (not extrapolated) across a gap longer than ~2.5 control periods.
- A fixed lookahead distance for the NEAR regime (mirroring the FAR regime's pure-pursuit),
  so lateral offset — not raw bearing — drives the correction, removing the depth-dependent gain
  blow-up.
- Heavier axis smoothing and a stability-weighted average feeding the frozen dock-normal
  reference carried across the FAR→NEAR hand-over via odometry.
- `visual_servo_kp` / `visual_servo_kd` were re-tuned for the new signal — the old values (tuned
  for the raw-bearing law) do not carry over.

## What did **not** change

- The FAR-regime pure-pursuit on the 3-tag averaged dock axis (`13_perception_and_line.md`).
- The hysteresis deadband mechanism and the sigma-delta PWM actuation floor
  (`_floor_omega` — a *different*, later attempt at a stiction fix on the *search scan*, not the
  NEAR corrector, was tried and reverted the same night; see
  `docs/2026-07-07-session-docking-corrector-rewrite.md` in the notes repo for that detail).
- `stop_lidar_in_approach` semantics (only the module comment describing why it defaults to
  `False` was corrected — it previously wrongly claimed AprilTag and the LiDAR driver shared a
  process container).
- The continuous-autofocus camera fix (`camera.launch.py`, `AfMode: 2`) is unrelated to this
  steering law but shipped in the same session/PR — it fixes NEAR-field image blur, not the
  oscillation described here.
