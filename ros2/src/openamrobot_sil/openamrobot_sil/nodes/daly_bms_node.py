"""ROS 2 node: minimal Daly BMS emulator on vcan_bms, fed by the lumped pack model."""
import json
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from openamrobot_sil import battery_model, config, model_params
from openamrobot_sil.canbus import open_bus
from openamrobot_sil.daly_bms import DalyBmsEmulator
from openamrobot_sil.nodes.common import publish_json, steady_timer


class HostClock:
    """Duck-typed clock (now, generation, paused) on host monotonic time for this node until the
    clock contract (PR #151) gives it a Gazebo-fed clock. The models only see this object."""

    def __init__(self):
        self.generation = 0
        self.paused = False

    def now(self):
        return time.monotonic()


class DalyNode(Node):
    def __init__(self):
        super().__init__('sil_daly_bms')
        cfg_path = self.declare_parameter('config_file', '').value
        cfg = config.load(cfg_path)
        b, bus_cfg = cfg['bms'], cfg['buses']['bms']
        self.bus = open_bus(bus_cfg['interface'])
        self.emu = DalyBmsEmulator(
            self.bus, bus_cfg['bms_address'], bus_cfg['host_address'], b['n_cells'],
            b['n_temps'], b['pack_voltage_v'], b['current_a'], b['soc_percent'],
            b['capacity_ah']).start()
        self.sim = None
        m = b.get('model') or {}
        if m.get('enabled', False):
            params = model_params.load(os.path.join(os.path.dirname(cfg_path),
                                                    m['parameters_file']))
            pack, charger, self.drive_est, loads = battery_model.from_config(
                b, cfg['drive']['wheel_radius'], params)
            power = None
            if (m.get('power') or {}).get('enabled', False):
                power = battery_model.power_from_config(b, params, pack, charger)
            self.clock = HostClock()
            self.sim = battery_model.PackSimulation(self.emu, pack, charger, loads,
                                                    m.get('current_sign_discharge', -1),
                                                    clock=self.clock, power=power,
                                                    drive=self.drive_est)
            self.signs = (cfg['drive']['left_sign'], cfg['drive']['right_sign'])
            self.drive_timeout_s = m['drive'].get('state_timeout_s', 0.5)
            self.drive_seen = None
            self.period_s = m.get('period_s', 0.1)
            self.create_subscription(String, '/sil/drive/emulator_state', self.on_drive, 10)
            steady_timer(self, self.period_s, self.on_model)
        self.state_pub = self.create_publisher(String, '/sil/bms/emulator_state', 10)
        self.create_subscription(String, '/sil/bms/inject', self.on_inject, 10)
        steady_timer(self, 0.2, self.on_state)
        self.get_logger().info('Daly BMS emulator (protocol unconfirmed) on %s, pack model %s' %
                               (bus_cfg['interface'], 'on' if self.sim else 'off'))

    def on_drive(self, msg):
        try:
            state = json.loads(msg.data)
        except ValueError:
            return
        now = self.clock.now()
        w = battery_model.wheel_rad_s_from_drive_state(state, *self.signs)
        self.sim.drive_w = self.drive_est.power_w(now, w, bool(state.get('enabled')))
        self.drive_seen = now

    def on_model(self):
        if (self.drive_seen is None or
                self.clock.now() - self.drive_seen > self.drive_timeout_s):
            self.sim.drive_w = 0.0  # no drive state: no drive load, said in the state output
        self.sim.tick()

    def on_state(self):
        out = self.emu.state()
        if self.sim:
            out.update(self.sim.state())
        publish_json(self.state_pub, out)

    def on_inject(self, msg):
        """Commands: failure <byte> <mask>, clear, mos <charge 0|1> <discharge 0|1>,
        soc <percent>, voltage <V>, current <A>, offline, online,
        step <A> (extra load current, model), charge on|off (bench charger, model),
        model on|off, switch <name> open|closed (power model input: disconnect,
        bms_discharge, bms_charge, k1, k2, kpre), load <name> <value> (power model rail load).
        voltage and current switch the model off so the value stays."""
        words = msg.data.split()
        try:
            cmd, args = words[0], words[1:]
            if cmd == 'failure':
                self.emu.inject_failure(int(args[0], 0), int(args[1], 0))
            elif cmd == 'clear':
                self.emu.clear_failures()
            elif cmd == 'mos':
                self.emu.set_mos(int(args[0]), int(args[1]))
                if self.sim and self.sim.power:
                    # The same input sets the reported MOS flags and the modelled switches.
                    self.sim.power.set_switch('bms_charge', int(args[0]) != 0)
                    self.sim.power.set_switch('bms_discharge', int(args[1]) != 0)
            elif cmd == 'soc':
                self.emu.set_values(soc_percent=float(args[0]))
                if self.sim:
                    self.sim.pack.soc_percent = min(100.0, max(0.0, float(args[0])))
            elif cmd == 'voltage':
                self._model_off()
                self.emu.set_values(pack_voltage_v=float(args[0]))
            elif cmd == 'current':
                self._model_off()
                self.emu.set_values(current_a=float(args[0]))
            elif cmd in ('offline', 'online'):
                self.emu.offline = cmd == 'offline'
            elif cmd in ('step', 'charge', 'model', 'switch', 'load') and self.sim is None:
                raise ValueError('pack model disabled in the configuration')
            elif cmd in ('switch', 'load') and self.sim.power is None:
                raise ValueError('power model disabled in the configuration')
            elif cmd == 'switch':
                if args[1] not in ('open', 'closed'):
                    raise ValueError('switch state is open or closed')
                self.sim.power.set_switch(args[0], args[1] == 'closed')
            elif cmd == 'load':
                self.sim.power.set_load(args[0], float(args[1]))
            elif cmd == 'step':
                self.sim.step_a = float(args[0])
            elif cmd == 'charge':
                self.sim.charger.on = args[0] == 'on'
                self.sim.charger.done = False
            elif cmd == 'model':
                self.sim.enabled = args[0] == 'on'
            else:
                raise ValueError('unknown command')
        except (IndexError, KeyError, ValueError) as exc:
            self.get_logger().warning('bad injection %r: %s' % (msg.data, exc))
            return
        self.get_logger().info('injected %s' % msg.data)

    def _model_off(self):
        if self.sim:
            self.sim.enabled = False

    def shutdown(self):
        self.emu.stop()
        self.bus.shutdown()


def main():
    rclpy.init()
    node = DalyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.shutdown()


if __name__ == '__main__':
    main()
