"""Odometry from the wheel speeds the firmware image reports (audit item SW-08).

In the Renode modes nothing integrated an odometry: the image prints wheel speeds and owns
no /odom. FwOdometry integrates [ch1, ch2] of /sil/fw/wheel_speeds into a pose. It is a
stand-in for the odometry the base controller will publish itself, so that the firmware_sil
profile has one writer for /odom and for the odom to base_footprint transform.

Rules:
  - A sample that is NaN (the image has no valid value, or its console is stale) moves
    nothing and is counted. An unknown speed is not a speed of 0: the pose keeps its last
    value and `valid` is False, so a consumer can tell "standing" from "not known".
  - Time is the receipt time of the sample (I8 rev 4: freshness by local receipt time). The
    image's own stamps are Renode virtual time, which lags wall time, and the wheels turn in
    Gazebo at wall rate. A gap longer than max_gap_s is not integrated.
  - ch1 is the left wheel and ch2 the right one, both positive forwards: SIL test values
    (sil.yaml left_sign and right_sign). Axis order and sign of the real drive are open.
Coarse by design: the image reports 10 samples per virtual second.
"""
import math


class FwOdometry:
    def __init__(self, wheel_radius, wheel_separation, left_sign=1, right_sign=1,
                 max_gap_s=1.0):
        self.r, self.sep = wheel_radius, wheel_separation
        self.signs = (left_sign, right_sign)
        self.max_gap_s = max_gap_s
        self.x = self.y = self.yaw = 0.0
        self.v = self.w = 0.0
        self.valid = False
        self.samples = self.unknown_samples = self.gaps = 0
        self._last_t = None

    def update(self, ch1, ch2, t):
        """Take one wheel speed sample (rad/s) received at time t (s). Returns valid."""
        self.samples += 1
        known = not (math.isnan(ch1) or math.isnan(ch2))
        if not known:
            self.unknown_samples += 1
            self.valid = False
            self.v = self.w = math.nan
            self._last_t = None        # nothing is integrated across an unknown stretch
            return False
        left, right = ch1 * self.signs[0] * self.r, ch2 * self.signs[1] * self.r
        v, w = (left + right) / 2.0, (right - left) / self.sep
        if self._last_t is not None:
            dt = t - self._last_t
            if 0.0 < dt <= self.max_gap_s:
                self.x += v * dt * math.cos(self.yaw + w * dt / 2.0)
                self.y += v * dt * math.sin(self.yaw + w * dt / 2.0)
                self.yaw += w * dt
            elif dt > self.max_gap_s:
                self.gaps += 1
        self._last_t = t
        self.v, self.w, self.valid = v, w, True
        return True

    def state(self):
        return {'x': self.x, 'y': self.y, 'yaw': self.yaw, 'valid': self.valid,
                'samples': self.samples, 'unknown_samples': self.unknown_samples,
                'gaps': self.gaps}
