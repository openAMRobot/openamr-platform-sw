# SPDX-License-Identifier: MIT
"""Write a copy of a Nav2 parameter file with test overrides, for one trial run. The source file is not changed.

  python3 -m openamrobot_clearance_trials.overrides <in.yaml> <out.yaml> '<JSON object>'

Keys are paths separated by "/" (parameter names contain dots), for example
  {"local_costmap/local_costmap/ros__parameters/inflation_layer/inflation_radius": 0.45}
A value of null removes the key. Every path but the last element must exist already, so that a typing mistake
fails instead of adding a parameter nobody reads; a new last element is allowed (a new polygon, for example).
"""
import json
import sys

import yaml


def apply(tree, overrides):
    changed = []
    for path, value in overrides.items():
        parts = path.split("/")
        node = tree
        for part in parts[:-1]:
            if not isinstance(node, dict) or part not in node:
                raise KeyError(f"{path}: '{part}' does not exist")
            node = node[part]
        if not isinstance(node, dict):
            raise KeyError(f"{path}: parent is not a mapping")
        before = node.get(parts[-1], "<absent>")
        if value is None:
            node.pop(parts[-1], None)
        else:
            node[parts[-1]] = value
        changed.append((path, before, value))
    return changed


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    source, target, text = argv
    with open(source, encoding="utf-8") as f:
        tree = yaml.safe_load(f)
    overrides = json.loads(text) if text.strip() else {}
    for path, before, value in apply(tree, overrides):
        print(f"override {path}: {before!r} -> {value!r}")
    with open(target, "w", encoding="utf-8") as f:
        yaml.safe_dump(tree, f, default_flow_style=None, sort_keys=False, width=200)
    print(f"wrote {target} ({len(overrides)} overrides)")


if __name__ == "__main__":
    main()
