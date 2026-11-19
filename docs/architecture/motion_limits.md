# Base motion limits per command layer

Status: Proposed, 19 Nov 2026. Configuration record, read on main 69847ab. Not a stop test and not
safety evidence. Test: `ros2/src/openamrobot_sil/test/test_motion_limit_layers.py`. Tracking issue: #172.

Declared in openAMRobot/.github `decisions.yaml` (87c8486): SPEED-CEILING, base speed at most
0.5 m/s in Nav2, the smoother, the ZLAC8015D and Gazebo; MASS-LEDGER (D-29), deceleration 1.5 m/s2
travel and 1.0 m/s2 hold-and-move, identical in those four layers, in a separate change after #90.

| Layer | Speed m/s | Reverse m/s | Deceleration m/s2 | Configured in | Meets the ceiling |
|---|---|---|---|---|---|
| Nav2 controller (DWB FollowPath) | 0.20 | 0.05 | 2.5 | `openamrobot_nav2/config/nav2_params.yaml` | speed yes, deceleration no |
| Velocity smoother | 0.20 | 0.20 | 2.5 | same file | speed yes, deceleration no |
| Docking sequencer | 0.10 | 0.10 | none | `openamrobot_docking/config/dock_trigger.yaml` | speed yes; publishes `/cmd_vel` directly, around the smoother |
| Gazebo DiffDrive | 1.0 | 1.0 | 0.5 | `openamrobot_description/urdf/gazebo_control.xacro` | no |
| ZLAC8015D emulator | 10.47 | 10.47 | 20.9 (quick stop 6085h: 10 ms per 1000 r/min) | vendored `zlac8015d_emulator.py` (60FFh 1000 r/min, 6084h 500 ms), `openamrobot_sil/config/sil.yaml` wheel radius 0.100 | no |

No layer has a hold-and-move mode, so the 1.0 m/s2 value is not compared. The ZLAC8015D ramp
reference of 6083h and 6084h is unconfirmed in the emulator itself. The rows that break the
ceiling are listed in the test with their current values; changing a configured value, or
fixing one, fails the test until the table and the test are updated with it.
