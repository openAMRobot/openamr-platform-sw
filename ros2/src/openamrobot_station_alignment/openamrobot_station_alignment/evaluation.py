# SPDX-License-Identifier: MIT
"""Pass criteria and the summary table of the station alignment trials. No ROS import."""
REACH_XY, REACH_YAW = 0.04, 0.08
CHECKER_XY, CHECKER_YAW = 0.025, 0.05
START_POSES = ((0.0, 0.0, 0.0), (-1.2, -0.9, 0.8), (1.2, -0.8, 2.6), (0.5, -0.5, -0.5), (-0.6, -0.3, 3.0),
               (0.9, 0.3, -2.2), (-1.0, 0.2, 1.6))
# Short correction regression (platform-sw #192): a station goal sent from a pose this far beside the station and
# this far off its heading. Less than the 0.05 m the general progress checker needs, more than the 0.3 rad at which
# the station controller turns in place first. Test inputs, not tolerances.
SHORT_CORRECTION_LEFT, SHORT_CORRECTION_YAW = 0.03, 0.40
SHORT_CORRECTION_MAX_XY, SHORT_CORRECTION_MIN_YAW = 0.05, 0.30
COMMAND_PUBLISHERS = {"/cmd_vel": {"collision_monitor"},
                      "/cmd_vel_nav": {"controller_server", "behavior_server"}}


def evaluate(trials, n_required):
    """(passed, reasons) of the trial records."""
    reasons = []
    done = [t for t in trials if t.get("aligned") and t["aligned"].get("error_xy") is not None]
    if len(done) < n_required:
        reasons.append(f"{len(done)} of {n_required} trials completed")
    for t in done:
        a = t["aligned"]
        if a["error_xy"] > REACH_XY or a["error_yaw"] > REACH_YAW:
            reasons.append(f"trial {t['trial']}: aligned error {a['error_xy']:.4f} m {a['error_yaw']:.4f} rad "
                           "beyond the reach budget")
    return not reasons, reasons


def command_publishers_ok(publishers):
    bad = []
    for topic, allowed in COMMAND_PUBLISHERS.items():
        names = set(publishers.get(topic, []))
        extra = names - allowed
        if extra:
            bad.append(f"{topic} published by {sorted(extra)}")
    if not publishers.get("/cmd_vel"):
        bad.append("/cmd_vel has no publisher")
    return not bad, bad


def summary(trials):
    def stats(key, field):
        values = [t[key][field] for t in trials if t.get(key) and t[key].get(field) is not None]
        if not values:
            return "n/a"
        return f"mean {sum(values) / len(values):.4f}, max {max(values):.4f} (n={len(values)})"
    lines = ["| Trial | #164 alone xy (m) | yaw (rad) | AMCL - truth (m) | marker - truth xy (m) | yaw (rad) |"
             " + marker xy (m) | yaw (rad) | corrections |",
             "|---|---|---|---|---|---|---|---|---|"]
    f = lambda v: "n/a" if v is None else f"{v:.4f}"  # noqa: E731
    for t in trials:
        b, a = t.get("baseline") or {}, t.get("aligned") or {}
        lines.append(f"| {t['trial']} | {f(b.get('error_xy'))} | {f(b.get('error_yaw'))} | {f(b.get('amcl_error_xy'))}"
                     f" | {f(b.get('marker_error_xy'))} | {f(b.get('marker_error_yaw'))} | {f(a.get('error_xy'))}"
                     f" | {f(a.get('error_yaw'))} | {a.get('corrections', 'n/a')} |")
    lines += ["", f"#164 alone, position: {stats('baseline', 'error_xy')}",
              f"#164 alone, yaw: {stats('baseline', 'error_yaw')}",
              f"#164 + marker, position: {stats('aligned', 'error_xy')}",
              f"#164 + marker, yaw: {stats('aligned', 'error_yaw')}",
              f"AMCL - ground truth at the station: {stats('baseline', 'amcl_error_xy')}",
              f"marker estimate - ground truth: {stats('baseline', 'marker_error_xy')}; yaw "
              f"{stats('baseline', 'marker_error_yaw')}"]
    return "\n".join(lines)


def count_corrections(report):
    """Corrections the alignment ran: iterations that planned a correction (they carry "target_odom" and
    "moves", written by station_alignment_node._correct). Was counted on a "corrected_goal" key the node no
    longer writes, so every trial reported 0 (Kunal's review of aa8fd11, point 1)."""
    return sum(1 for step in (report or {}).get("iterations", []) if "target_odom" in step)


def short_correction_offset(index):
    """(along, left, yaw) of the pre-pose of repetition `index` in the station frame; the side alternates."""
    sign = 1.0 if index % 2 == 0 else -1.0
    return (0.0, sign * SHORT_CORRECTION_LEFT, sign * SHORT_CORRECTION_YAW)


def short_correction_exercised(record):
    """True when the repetition started where the regression needs it: the controller's own pose (AMCL) less than
    SHORT_CORRECTION_MAX_XY from the station goal and more than SHORT_CORRECTION_MIN_YAW off its heading."""
    start = record.get("start") or {}
    xy, yaw = start.get("amcl_goal_xy"), start.get("amcl_goal_yaw")
    return (record.get("pre_nav2") == "succeeded" and xy is not None and yaw is not None
            and xy < SHORT_CORRECTION_MAX_XY and yaw > SHORT_CORRECTION_MIN_YAW)


def evaluate_short_corrections(records, n_required):
    """(passed, reasons). Every repetition that started in the regression's region must end with Nav2 "succeeded"
    and the ground truth inside the reach budget (the limits of evaluate()), and there must be n_required of them.
    A repetition that did not start there is listed and proves nothing either way."""
    reasons = []
    exercised = [r for r in records if short_correction_exercised(r)]
    if len(exercised) < n_required:
        reasons.append(f"{len(exercised)} of {n_required} repetitions started inside the regression's region")
    for r in exercised:
        if r.get("nav2") != "succeeded":
            reasons.append(f"repetition {r['repetition']}: Nav2 {r.get('nav2')}")
        elif r.get("error_xy") is None or r["error_xy"] > REACH_XY or r["error_yaw"] > REACH_YAW:
            reasons.append(f"repetition {r['repetition']}: ended {r.get('error_xy')} m {r.get('error_yaw')} rad, "
                           "beyond the reach budget")
    return not reasons, reasons


def short_correction_summary(records):
    lines = ["| Repetition | pre-pose Nav2 | start to goal xy (m), AMCL | yaw (rad) | exercised | Nav2 | sim s |"
             " yaw travelled (rad) | path travelled (m) | end xy (m), truth | yaw (rad) |",
             "|---|---|---|---|---|---|---|---|---|---|---|"]
    f = lambda v: "n/a" if v is None else f"{v:.4f}"  # noqa: E731
    for r in records:
        start = r.get("start") or {}
        lines.append(f"| {r['repetition']} | {r.get('pre_nav2')} | {f(start.get('amcl_goal_xy'))} |"
                     f" {f(start.get('amcl_goal_yaw'))} | {'yes' if short_correction_exercised(r) else 'no'} |"
                     f" {r.get('nav2')} | {f(r.get('sim_s'))} | {f(r.get('yaw_travel'))} | {f(r.get('xy_travel'))} |"
                     f" {f(r.get('error_xy'))} | {f(r.get('error_yaw'))} |")
    return "\n".join(lines)
