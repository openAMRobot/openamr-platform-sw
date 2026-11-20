# Copyright 2026 OpenAMRobot contributors
# SPDX-License-Identifier: MIT
"""Odometry of the host client node publishes only new 606Ch samples (review of #169). Needs
rclpy (runs in CI sil-tests)."""
import types

from openamrobot_sil.nodes import stm32_host_client_node as nodes


def test_odometry_node_publishes_once_per_new_sample_and_never_repeats_the_last(monkeypatch):
    """HostClientNode.on_odom on a fake node: one message per new 606Ch sample, none on the
    ticks between. revert: on_odom publishes the held twist on every tick."""
    published = []
    drive = types.SimpleNamespace(samples=0)
    state = {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'v': 0.1, 'w': 0.05}

    def new_sample(last):
        return dict(state, samples=drive.samples) if drive.samples > last else None

    def full_state():
        return dict(state, samples=drive.samples)
    drive.new_sample, drive.state = new_sample, full_state
    clock = types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: None))
    fake = types.SimpleNamespace(drive=drive, _odom_samples=0,
                                 odom_pub=types.SimpleNamespace(publish=published.append),
                                 get_clock=lambda: clock)
    monkeypatch.setattr(nodes, 'Odometry', lambda: types.SimpleNamespace(
        header=types.SimpleNamespace(stamp=None, frame_id=''), child_frame_id='',
        pose=types.SimpleNamespace(pose=types.SimpleNamespace(
            position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
            orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))),
        twist=types.SimpleNamespace(twist=types.SimpleNamespace(
            linear=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
            angular=types.SimpleNamespace(x=0.0, y=0.0, z=0.0)))), raising=False)
    on_odom = nodes.HostClientNode.on_odom
    on_odom(fake)
    assert published == []                                   # no sample yet
    drive.samples = 1
    for _ in range(5):
        on_odom(fake)
    assert len(published) == 1                               # the drive answered once
    drive.samples = 3
    on_odom(fake)
    on_odom(fake)
    assert len(published) == 2
