# SPDX-License-Identifier: MIT
# Minimal stand-ins for the ROS modules station_alignment_node imports, so that its methods can be driven
# without ROS. Only what the reviewed methods touch is real; everything else is a placeholder.
import os
import sys
import types

# ros2/src: the source of openamrobot_nav2 stands in for its installed share directory (behavior_trees/, config/).
SRC = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


class _Any:
    def __init__(self, *a, **k):
        pass

    def __call__(self, *a, **k):
        return _Any()

    def __getattr__(self, name):
        return _Any()


def _mod(name, **attrs):
    m = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(m, k, v)
    sys.modules[name] = m
    return m


class TransformException(Exception):
    pass


class PackageNotFoundError(KeyError):
    pass


def get_package_share_directory(name):
    """openamrobot_nav2 from the source tree; any other package as before ("/nonexistent")."""
    if name == "openamrobot_nav2":
        return os.path.join(SRC, name)
    return "/nonexistent"


class Node:
    def __init__(self, *a, **k):
        pass


class GoalResponse:
    ACCEPT, REJECT = 1, 2


class CancelResponse:
    ACCEPT, REJECT = 1, 2


class Result:
    def __init__(self):
        self.error_code = 0
        self.error_msg = ""


class StopState:
    STATE_LOCKED, STATE_RELEASE_REFUSED, STATE_RUNNING = 0, 1, 2


class NavigateToPose:
    Result = Result
    Goal = _Any


def install():
    _mod("rclpy", ok=lambda: True, init=lambda **k: None, shutdown=lambda: None)
    _mod("rclpy.action", ActionClient=_Any, ActionServer=_Any, CancelResponse=CancelResponse,
         GoalResponse=GoalResponse)
    _mod("rclpy.callback_groups", ReentrantCallbackGroup=_Any)
    _mod("rclpy.executors", MultiThreadedExecutor=_Any)
    _mod("rclpy.node", Node=Node)
    _mod("rclpy.qos", DurabilityPolicy=_Any(), QoSProfile=_Any, ReliabilityPolicy=_Any())
    _mod("rclpy.time", Time=_Any)
    _mod("rclpy.parameter", Parameter=_Any)
    _mod("ament_index_python")
    _mod("ament_index_python.packages", get_package_share_directory=get_package_share_directory,
         PackageNotFoundError=PackageNotFoundError)
    _mod("geometry_msgs")
    _mod("geometry_msgs.msg", PoseWithCovarianceStamped=_Any)
    _mod("nav_msgs")
    _mod("nav_msgs.msg", Odometry=_Any)
    _mod("nav2_msgs")
    _mod("nav2_msgs.action", BackUp=_Any, DriveOnHeading=_Any, NavigateToPose=NavigateToPose)
    _mod("openamr_nav_msgs")
    _mod("openamr_nav_msgs.msg", StopState=StopState)
    _mod("std_msgs")
    _mod("std_msgs.msg", String=lambda data="": types.SimpleNamespace(data=data))
    _mod("tf2_ros", Buffer=_Any, TransformException=TransformException, TransformListener=_Any)
    _mod("builtin_interfaces")
    _mod("builtin_interfaces.msg", Duration=_Any)
