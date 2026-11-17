# Copyright 2026 OpenAMRobot contributors
# SPDX-License-Identifier: MIT
"""Device behaviour models of the digital twin (hardware behaviour layer, Gate D preparation).

A model here converts commands plus the electronics behaviour of one device into an effort or an
owned control interface for Gazebo, and derives its feedback from the plant state it is given.
Gazebo (DART) owns the motion. Every model takes an injected clock (openamrobot_sil.sim_time) and
never reads host time. Fidelity F1 (datasheet-informed) at most. Nothing here is wired into a
launch file or a Gazebo scene yet.
"""
