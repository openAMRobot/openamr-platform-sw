# Integrated Digital Twin Baseline 1: platform-sw integration branch

Branch `integration/baseline1`, created 17 Nov 2026 from `main` at `69847ab`. Every PR was merged with
`git merge --no-ff` (no rebase, no force-push) at the head SHA verified on GitHub immediately before the merge.
This branch is never merged into `main`: each PR keeps its own review. Simulation evidence only.

| Order | PR | Branch | Head SHA | Merge commit | Conflicts | Gate status |
|---|---|---|---|---|---|---|
| 1 | #152 | ci/repin-gz-vendor-2026-11-16 | `546743b` | `6d35912` | none | branch gate |
| 2 | #146 | fix/wheel-loaded-radius | `68642a8` | `0742c3d` | none (CHANGELOG.md, STATE.md, sil-topology.md merged automatically) | branch gate |
| 3 | #149 | claude/optimistic-planck-frfht7 | `e7f242e` | `d4c9f72` | none | branch gate |
| 4 | #151 | ws1/clock-contract | `fe071f4` | `e9e6481` | `sil-tests.yml` protocol count, `CHANGELOG.md` | branch gate |
| 5 | #158 | ws1/time-injection (stacked on #151) | `38728c8` | `d36b33a` | `sil-tests.yml` protocol count, `CHANGELOG.md` | branch gate |
| - | #141 | sil/renode-telemetry | `9913d87` | not merged | see below | excluded |
| - | #144 | sil/renode-uros (stacked on #141) | `53a69e9` | not merged | see below | excluded |
| - | #145 | claude/jolly-albattani-o25o3c (stacked on #144) | `8c5bd52` | not merged | see below | excluded |

## Conflicts and resolutions

- #151 into the branch after #149: `.github/workflows/sil-tests.yml` expected 77 protocol tests (#149) and 67 (#151).
  Set to 87, the count on the merged tree: main 57, plus 20 from #149, plus 10 from #151. `CHANGELOG.md`: both
  Unreleased entries kept, #149 first.
- #158 into the branch: `sil-tests.yml` expected 87 (this branch) and `(111, 3 xfail)` (#158). Set to `(131, 3)`.
  #158's count check that separates strict xfails from skips and its Gazebo count of 6 are kept. 131 is 126 tests
  collected without rclpy plus the 5 tests of `test_node_clock.py` (3 functions, one parametrized twice), which need
  rclpy and were counted from the source. `CHANGELOG.md`: both entries kept.

## Why #141, #144 and #145 are excluded

They are relevant: the Build 1 scene in openamr-upperbody-sw calls `sil.launch.py`'s
`can_nodes(config, stm32, gazebo, renode)` and `default_renode_args()`, which exist only from #141 onward, and it
builds `openamrobot_sil` from #145 `8c5bd52`. A trial merge of #141 onto this branch conflicts in nine files, five of
them code (`sil.launch.py`, `stm32_host.py`, `stm32_host_client_node.py`, `zlac8015d_node.py`, `zlac_gazebo.py`).
Both sides rewrite the drive host's step loop and bring-up: the clock contract (#151, #158) restarts the SDO
bring-up after a clock reset or stall, while #141's `command_only` mode has no SDO client and leaves the bring-up to
the firmware. Joining them decides how the drive is enabled again after a reset in Renode mode. That is a design
decision for the owners of the clock contract and of the Renode work, not a minimal conflict resolution, so it was
not made here.

Consequence for Baseline 1: the whole-robot acceptance run in openamr-upperbody-sw builds this branch's
description, Nav2, perception and docking, but `openamrobot_sil` from #145. The clock contract of #151 and #158 is
therefore not in the whole-robot run, and #145's `sil.yaml` still states `wheel_radius: 0.100` while this branch's
description and `sil.yaml` state the loaded radius 0.098 from #146. Both are open until #141 to #145 are reconciled
with #151 and #158.

## Verification on this branch (17 Nov 2026)

- `cd ros2/src/openamrobot_sil && SIL_TEST_BUS=virtual python3 -m pytest -q -p no:cacheprovider test
  --ignore=test/test_node_clock.py --ignore=test/test_sil_launch.py`: 118 passed, 3 xfailed (strict, #159 and #160).
  Python's in-process CAN bus, not vcan; no ROS on the machine, so the 10 tests of the two ignored files did not run.
- `python3 -m pytest --collect-only` of `test` with a collection-only stub for `launch_ros`: 126 collected, plus 5 in
  `test_node_clock.py` counted from the source: 131.
- Not run here: colcon build, vcan, Gazebo. CI on the branch head `e0dd1d7` (draft PR #161), 17 Nov 2026:
  `sil-tests` run 44076631488 passed (36 vendored, 131 protocol with 3 strict xfail, 6 Gazebo end-to-end, all on vcan);
  `description-tests` run 44076631476 passed (221 description tests; 14 friction tests on gz-sim 8.15.0 and sdformat
  14.9.0 with #152's vendor pins; 13 of 14 fail with the fix reverted, as intended); `Repository quality` passed.

## Branch gate status

"branch gate" in the table means: the merged tree passes the platform workflows above. It is not Baseline 1.
The whole-robot acceptance run (openamr-upperbody-sw draft PR #77, run 44076639872) on `d36b33a` did not pass:
3 of 8 phases passed. Record and classification: openamr-upperbody-sw `docs/integration/baseline1.md`.

Heads that moved after these merges (pinned here at the verified SHA, not re-merged): #158 is now `0b18678`.
