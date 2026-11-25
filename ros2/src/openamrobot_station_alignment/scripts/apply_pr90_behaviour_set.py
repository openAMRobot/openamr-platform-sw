#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Test only: put platform-sw #90's reverse-free behaviour set onto a copy of openamrobot_nav2 (#164's head) for the
station alignment simulation, so the correction is shown to work without reverse (D-19).

  apply_pr90_behaviour_set.py PR90_NAV2_PARAMS NAV2_PKG_DIR

From #90's nav2_params.yaml: behavior_server.behavior_plugins (spin and wait only; backup, drive_on_heading and
assisted_teleop removed), velocity_smoother.min_velocity (linear x 0) and controller_server.FollowPath.min_vel_x (0).
Into NAV2_PKG_DIR/config/nav2_params.yaml, in place. From every tree in NAV2_PKG_DIR/behavior_trees: the BackUp
recovery removed, as #90's *_no_backup.xml trees do; with #90's behavior_plugins a tree that names BackUp has no
backup server to call. Used by the CI workflow on its checkout only; no file of #164 or #90 changes in git.
"""
import glob
import os
import re
import sys

import yaml

BACKUP = re.compile(r"^\s*<BackUp\b[^>]*/>\s*\n", re.MULTILINE)


def apply(pr90_params, nav2_dir):
    with open(pr90_params, encoding="utf-8") as f:
        pr90 = yaml.safe_load(f)
    path = os.path.join(nav2_dir, "config", "nav2_params.yaml")
    with open(path, encoding="utf-8") as f:
        params = yaml.safe_load(f)
    changes = []
    for node, keys in (("behavior_server", ["behavior_plugins"]), ("velocity_smoother", ["min_velocity"]),
                       ("controller_server", ["FollowPath", "min_vel_x"])):
        src, dst = pr90[node]["ros__parameters"], params[node]["ros__parameters"]
        for k in keys[:-1]:
            src, dst = src[k], dst[k]
        old, dst[keys[-1]] = dst.get(keys[-1]), src[keys[-1]]
        changes.append(f"{node}.{'.'.join(keys)}: {old} -> {dst[keys[-1]]}")
    behaviors = params["behavior_server"]["ros__parameters"]
    for name in list(behaviors):
        plugin = isinstance(behaviors[name], dict) and "plugin" in behaviors[name]
        if plugin and name not in behaviors["behavior_plugins"]:
            del behaviors[name]
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(params, f, sort_keys=False)
    for tree in sorted(glob.glob(os.path.join(nav2_dir, "behavior_trees", "*.xml"))):
        with open(tree, encoding="utf-8") as f:
            text = f.read()
        stripped, n = BACKUP.subn("", text)
        if n:
            with open(tree, "w", encoding="utf-8") as f:
                f.write(stripped)
            changes.append(f"{os.path.basename(tree)}: {n} BackUp removed")
    return changes


if __name__ == "__main__":
    for line in apply(sys.argv[1], sys.argv[2]):
        print(line)
