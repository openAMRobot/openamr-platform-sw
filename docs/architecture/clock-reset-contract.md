# Clock, reset and stale-frame contract of the lift SIL path (Proposed)

Status: Proposed (17 Nov 2026). Owner of the contract: software lead. The CTL-017 wire contract
and firmware timer semantics stay with the platform lead; nothing here changes them. This page
refines the clock contract of [sil-topology.md section 6](sil-topology.md#6-clocks-contract-proposed)
(rules C1 to C6, gates D3) for the lift path. Development evidence only: emulators and a
lock-step stand-in for Gazebo. It is not real-robot, mechanical, electrical or safety evidence.

Tests: `ros2/src/openamrobot_sil/test/test_lift_fw_sil.py` (new) and
`ros2/src/openamrobot_sil/test/test_clock_gates.py`, `test_sim_time.py`, `test_node_clock.py`
(from the clock contract work). Below, `fw_sil::name` is a test of the first file, `gates::name`
of the second, `sim_time::name` and `node_clock::name` of the others.

## 1. The path

| Hop | Code |
|---|---|
| ROS 2 command `/sil/lift/command`, `/sil/lift/permit` | `openamrobot_sil/nodes/stm32_host_client_node.py` (`on_lift_command`, `on_permit`) |
| Base controller CAN3 role (stand-in): RPDO1, keep-alive, core heartbeat, NMT start, permit PDO | `openamrobot_sil/stm32_host.py` `LiftHost` |
| CAN3 | `vcan_aux` (`config/sil.yaml`), opened by `openamrobot_sil/canbus.py` `open_bus` |
| CTL-017 firmware interface: object dictionary, RPDO1 commands, faults F01 to F09, TPDO1/TPDO2 | vendored `vendor/can_sil/ctl017_emulator.py`, unmodified |
| Clock, permit gate and plant coupling | `openamrobot_sil/ctl017_gazebo.py` `GazeboCtl017` |
| Motor speed demand to Gazebo | `openamrobot_sil/nodes/ctl017_node.py` `on_cmd`, `/sil/lift/gz_cmd_vel`, `config/gz_bridge_sil.yaml` |
| Simulated actuator | `urdf/sil_robot.urdf.xacro`: `lift_joint`, gz-sim `JointController` (velocity mode) |
| Feedback | `JointStatePublisher` 100 Hz, bridge, `ctl017_node.py` `on_joints`, `GazeboCtl017.set_joint_position`, Hall count and stop sensors |
| Status back to ROS 2 | TPDO1/TPDO2 decoded by `LiftHost._rx`, `/sil/lift/status` |
| Clock | `/clock` bridged from Gazebo, `nodes/common.py` `NodeClock`, `sim_time.py` `ClockCoordinator` and `FedClock` |

## 2. Rules

Each rule is Proposed. "Node" is the CTL-017 emulator, "host" the base controller stand-in.

### R1 First clock and startup ordering

Before the first `/clock` the node is not powered: it sends no frame (no boot-up, no heartbeat),
drops every received frame (NMT, SDO, PDO, permit) and runs no timer. It powers up at the first
clock time: every timer starts there, then the boot-up frame. The host sends nothing before the
first clock. Any start order of Gazebo, bridge, node and host gives the same state.
Tests: `fw_sil::test_lift_node_thread_sends_no_frame_before_the_first_clock`,
`fw_sil::test_frames_received_before_the_first_clock_are_not_kept`,
`gates::test_before_the_first_clock_the_lift_node_accepts_no_command`,
`gates::test_host_stand_in_sends_nothing_before_the_first_clock`,
`sim_time::test_no_stall_before_the_first_clock_message`.
The drive emulator does not meet R1 yet (#160, strict xfail
`gates::test_before_the_first_clock_the_drive_sends_no_heartbeat_and_takes_no_enable`).

### R2 Permit absent, then present

The permit is a functional input of the node (sil-topology.md section 3.1). Without a fresh
permit a move or homing command is refused; a refused command is never executed later, also not
by its keep-alive; motion needs a new command after the permit arrives.
Test: `fw_sil::test_permit_absent_refuses_present_allows_and_the_refused_command_is_not_replayed`.

### R3 Motion to a target, feedback within tolerance

A move ends at the commanded stop with no fault; the position the host reads in TPDO1 and the
joint position both lie within the stop tolerance (0x2101:03, 2.0 mm); the travel time matches
18 mm/s on simulation time. Joint states at 100 Hz that arrive up to 50 ms late raise no F02 or
F03. Tests: `fw_sil::test_move_reaches_the_stop_and_can_feedback_agrees_within_tolerance`,
`fw_sil::test_joint_feedback_at_100_hz_with_transport_lag_raises_no_f02_and_arrives`,
`gates::test_lift_rate_window_does_not_depend_on_runner_speed`.

### R4 Cancel mid-motion

STOP halts the drive at once, drops the target, raises no fault; its keep-alive restarts
nothing; a new command moves again.
Test: `fw_sil::test_stop_command_mid_motion_halts_drops_the_target_and_nothing_restarts`.

### R5 Rewind and reset: a new generation

Simulation time moving backward is a new clock generation and a power cycle of every emulated
device: boot-up frame, PRE-OPERATIONAL, target and permit dropped, unhomed unless at a stop
sensor. The node's receive queue is lost with the power cycle, so no frame received before the
reset acts after it, whatever its order (NMT start included). The host takes a new permit boot
id, so a permit frame of the old boot that arrives after a frame of the new boot is not trusted.
Tests: `fw_sil::test_reset_is_a_new_generation_and_frames_queued_before_it_do_not_act`,
`fw_sil::test_a_permit_frame_delayed_across_a_reset_does_not_validate_the_permit`,
`gates::test_time_reset_is_a_power_cycle_of_the_lift_node`,
`gates::test_time_reset_at_a_stop_sensor_is_homed_again`,
`gates::test_frames_queued_before_a_reset_meet_the_power_cycled_lift_node`,
`sim_time::test_backward_feed_is_a_new_generation`.
Open (gap G3, #159): a frame delayed in transport past the node's restart carries no
generation and is accepted (strict xfail `gates::test_a_lift_command_delayed_across_a_reset_is_rejected`).

### R6 Stale frames are judged by receipt time

Freshness is the age since receipt in the node's own domain (C6). A permit received 190 ms ago
lets a command run; at 200 ms (SIL value, `lift.permit.timeout_ms`) the lift halts without a
fault and a command is refused. Without the permit gate, F07 is raised when the RPDO1 keep-alive
(0x1400:05) or the core heartbeat (0x1016:01) is 300 ms old, whichever expires first. A late
frame is fresh only from its receipt and never revives a refused command.
Tests: `fw_sil::test_permit_age_boundary_on_simulation_time`,
`fw_sil::test_stale_keep_alive_without_the_permit_gate_is_f07_at_the_deadline`,
`gates::test_delayed_permit_frames_are_judged_by_receipt_time`.

### R7 Repeated and delayed clock

The same `/clock` time twice is neither an advance nor a reset: no generation change, no extra
motion time. A `/clock` delayed on steady time changes nothing up to the 1 s stall bound and is
one stall above it (R9). Tests: `fw_sil::test_every_clock_time_repeated_counts_no_extra_motion_time`,
`fw_sil::test_delayed_clock_is_a_stall_only_above_the_bound`,
`gates::test_duplicate_clock_messages_change_no_generation_and_stop_no_lift_move`,
`gates::test_a_paused_gazebo_republishing_its_time_is_still_a_stall`,
`sim_time::test_delayed_clock_below_the_stall_bound_is_no_stall`,
`sim_time::test_delayed_clock_above_the_stall_bound_is_one_stall_and_no_jump_on_resume`.

### R8 Forward jump

A forward step is never a new generation. Device timers see the elapsed time: a permit or
keep-alive that expires in it acts as on the device (C3), also for a step below the 1 s jump
report bound. Steps above the bound are reported next to any fault they let expire. Nothing
restarts by itself. Tests: `fw_sil::test_forward_step_below_the_jump_bound_still_expires_the_permit`,
`gates::test_forward_jump_expires_the_lift_permit_without_a_fault_and_nothing_restarts`,
`gates::test_forward_jump_without_the_permit_gate_trips_f07_and_the_jump_is_reported`,
`sim_time::test_forward_jump_is_reported_never_a_new_generation_and_never_makes_a_stamp_fresh`.

### R9 Pause, clock loss and the simulation health hold

Below the stall bound a pause freezes every node timer. Above it (1 s steady time, C5) one stall
is reported ("simulation stalled", wall time and last simulation time, never a device fault):
the node stops the drive and invalidates the permit at the stall and again at the resume, the
host drops its command; nothing moves from a command given before the stall. The health
callback's only effect is the simulation-only hold: one zero demand on the Gazebo plant topic
per stall. It sends no CAN frame, changes no device state and is not a second command authority.
Tests: `gates::test_gazebo_pause_freezes_the_lift_timers`,
`gates::test_pause_longer_than_the_stall_bound_drops_motion_and_permit_and_nothing_rearms`,
`gates::test_a_permit_frame_received_during_the_pause_does_not_survive_the_resume`,
`gates::test_clock_loss_with_a_lift_command_buffered_before_the_stall_moves_nothing`,
`node_clock::test_health_callback_only_effect_is_the_simulation_only_plant_hold` (needs ROS 2).

### R10 Reconnection

After a ros_gz_bridge restart the lift is halted (permit expiry below the stall bound, the stall
above it), the Hall position agrees with the joint within the stop tolerance, and motion needs a
new command. After a host restart the permit expires (halt, no fault), the new host has a new
boot id and nothing to replay. After a node restart nothing is kept.
Tests: `fw_sil::test_reconnection_after_a_bridge_restart_stops_and_feedback_stays_consistent`,
`fw_sil::test_reconnection_after_a_host_stand_in_restart_needs_a_new_command`,
`gates::test_emulator_restart_keeps_nothing_and_the_host_does_not_replay`.
Open (G4, G5): see section 3.

## 3. Gaps

| ID | Gap | Where | Owner, tracking |
|---|---|---|---|
| G1 | The plant is a velocity command of gz-sim `JointController` on DART. The demand at full duty is 0.018 m/s, equal to the `lift_joint` velocity limit, and L1000 and L1350 sit at the joint limits 0 and 0.350 m. gz-sim issue 1684 reports DART ignoring a velocity command close to the velocity limit and joints stuck at a position limit. Not observed: the hosted Gazebo test moves the lift up from 0 m to L1175 and passed on 38728c8. Not excluded either; not run here (no Gazebo). The effort coupling of #155 is the proposed replacement | `urdf/sil_robot.urdf.xacro`, `ctl017_node.py` | software lead, #155 |
| G2 | The node thread tick, the `dt` of each step and the heartbeat producer period of the vendored `canopen_common` run on host time after the first clock; CTL-017 timers are evaluated on a 5 ms host tick | vendored `canopen_common.py` | platform lead, openamr-platform-fw #57 |
| G3 | A frame delayed in transport past a reset carries no generation: an RPDO1 runs, a permit frame of the old boot arriving before the first frame of the new boot is trusted for one period | CTL-017 v1 wire contract | platform lead, #159 |
| G4 | During a bridge outage Gazebo keeps the last lift demand (`JointController` has no command timeout) and the hold of R9 goes through the same bridge, so the lift moves at up to 18 mm/s for the outage. The node reports it as F08 (motion after stop) once the outage exceeds the stall bound | plant, bridge | software lead |
| G5 | RPDO1 has no producer boot id: after a host restart, a first command with the last sequence number is taken as a keep-alive (same op and stop, silently not executed) or refused with a sequence conflict | CTL-017 v1 wire contract | platform lead, #159 (strict xfail `fw_sil::test_the_first_command_of_a_restarted_host_is_executed_even_with_the_last_seq`) |
| G6 | Joint feedback late by 300 ms or more stops the Hall pulses for the 0x2101:19 stall time and raises F03; up to 200 ms late it arrives without a fault (lock-step probe, 17 Nov 2026) | bridge and executor latency | software lead, reported only |

## 4. Proposed for the CTL-017 contract owner (not implemented)

A producer epoch in the command path closes G3 and G5. Example, a proposal only: RPDO1 byte 3
`producer_boot` (u8, the low byte of the permit boot id, never 0) with these expectations, each a
test to write once the contract change request is accepted:

- a frame whose `producer_boot` differs from the last accepted one is not executed and resets
  the sequence tracking, so the next frame of the new boot is a new command;
- the first RPDO1 after the node's boot-up is accepted only after a permit frame of the same boot;
- a frame of an older boot that arrives after a newer one is refused with its own result code.

It changes the CTL-017 v1 dictionary and I8, so it goes through a contract change request
(openamr-upperbody-fw #15, openamr-platform-fw #20).
