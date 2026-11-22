# Copyright 2026 OpenAMRobot contributors
#
# Licensed under the MIT License (see LICENSE).

"""
Command timeout for the simulated base (#79), without ROS.

Gazebo's DiffDrive system keeps the last velocity command forever. The real base controllers do
not: the Teensy firmware stops after 200 ms without a fresh /cmd_vel (platform-fw firmware.ino,
docs/safety/motion-safety.md), and the Build 1 STM32 replaces a /cmd_vel older than 200 ms with
zero, non-latching (platform-fw #17, T11). This class decides, from simulation time only, when the
simulated base has to be held at zero. Simulation only: it is not the firmware's watchdog and not a
safety function.
"""

DEFAULT_TIMEOUT_S = 0.2
# Clock times are float seconds from integer nanoseconds; 10.2 - 10.0 is 0.19999999999999929.
# An age within a nanosecond of the timeout counts as reaching it.
_EPS_S = 1e-9
_CLOCK_RESET_EPS_S = 1e-6  # ignore sub-microsecond backward jitter
# How often zero is repeated while the command is stale. One zero already stops DiffDrive; the
# repeat is for a bridge that was not running when the timeout fired, and it must stay shorter
# than the timeout itself.
DEFAULT_ZERO_REPEAT_S = 0.1


def validate_tick_period(timeout_s, tick_period_s):
    """Reject a timer period that could consume the entire timeout budget."""
    if not 0.0 < tick_period_s < timeout_s:
        raise ValueError(
            f'tick_period_s must be in (0, timeout_s), got {tick_period_s!r} '
            f'for timeout_s={timeout_s!r}')


def validate_zero_repeat_period(timeout_s, tick_period_s, repeat_s):
    """Reject a repeat period the timer cannot honour or that outlasts the timeout."""
    if not tick_period_s <= repeat_s <= timeout_s:
        raise ValueError(
            f'zero_repeat_period_s must be in [tick_period_s, timeout_s] = '
            f'[{tick_period_s!r}, {timeout_s!r}], got {repeat_s!r}')


def zero_is_due(now_s, last_zero_s, repeat_s):
    """
    Whether the held zero has to be sent again.

    True before any zero was sent, once `repeat_s` has passed, and when the clock went backwards
    (a simulation reset): comparing against a zero from a later clock time would otherwise
    suppress zeros for as long as the reset went back.
    """
    if last_zero_s is None:
        return True
    elapsed = now_s - last_zero_s
    return elapsed < 0.0 or elapsed >= repeat_s - _EPS_S


class CommandTimeout:
    """
    Decide when the simulated base command is stale.

    Times are seconds of the node clock (simulation time in the simulator). Non-latching: the next
    command after a timeout passes through unchanged. A clock that goes backwards (a
    simulation reset) by more than a microsecond counts as stale, so a command from before the
    reset never keeps the base moving.
    """

    def __init__(self, timeout_s=DEFAULT_TIMEOUT_S):
        if not timeout_s > 0.0:
            raise ValueError(f'timeout_s must be positive, got {timeout_s!r}')
        self.timeout_s = float(timeout_s)
        self.last_command_s = None
        self.stale = False
        self.timeouts = 0

    def on_command(self, now_s):
        """Record a fresh command; return True when it ends a timeout (motion resumes)."""
        resumed = self.stale
        self.last_command_s = now_s
        self.stale = False
        return resumed

    def on_tick(self, now_s):
        """
        Return (hold_zero, newly_stale) for the current time.

        hold_zero is True while the last command is older than the timeout (or from before a clock
        reset); the caller then sends zero. newly_stale is True only on the tick where that starts.
        Before the first command there is nothing to stop and nothing is sent.
        """
        if self.last_command_s is None:
            return False, False
        age = now_s - self.last_command_s
        if age < -_CLOCK_RESET_EPS_S or age >= self.timeout_s - _EPS_S:
            newly = not self.stale
            if newly:
                self.timeouts += 1
            self.stale = True
            return True, newly
        return False, False

    def age(self, now_s):
        """Seconds since the last command, or None before the first one."""
        return None if self.last_command_s is None else now_s - self.last_command_s


def describe_stale(age_s, timeout_s):
    """
    Say why the base is held at zero, for the log.

    A negative age is a clock that went backwards (a simulation reset), not a command "-5 s"
    old; it is reported as such.
    """
    if age_s < 0.0:
        return (f'node clock went back {-age_s:.3f} s since the last /cmd_vel '
                '(simulation reset?): holding the simulated base at zero')
    return (f'no /cmd_vel for {age_s:.3f} s of node time (timeout {timeout_s:.3f} s): '
            'holding the simulated base at zero')
