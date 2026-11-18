# SPDX-License-Identifier: MIT
"""Pass criteria and the summary table of the station alignment trials. No ROS import."""
REACH_XY, REACH_YAW = 0.04, 0.08
CHECKER_XY, CHECKER_YAW = 0.025, 0.05
START_POSES = ((0.0, 0.0, 0.0), (-1.2, -0.9, 0.8), (1.2, -0.8, 2.6), (0.5, -0.5, -0.5), (-0.6, -0.3, 3.0),
               (0.9, 0.3, -2.2), (-1.0, 0.2, 1.6))
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
