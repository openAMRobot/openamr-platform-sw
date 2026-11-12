# Complete ROS 2 Course for Beginners
### From Zero to Robot Developer — Python & C++

---

> **Prerequisites:** Linux (Ubuntu 24.04 recommended), basic Python or C++ knowledge, terminal familiarity.
> **ROS 2 Version:** Jazzy Jalisco (LTS)

---

## Table of Contents

1. [Course Overview](#1-course-overview)
2. [Environment Setup](#2-environment-setup)
3. [Module 1 — ROS 2 Fundamentals](#module-1--ros-2-fundamentals)
4. [Module 2 — Topics & Communication](#module-2--topics--communication)
5. [Module 3 — Services & Parameters](#module-3--services--parameters)
6. [Module 4 — Launch Files & Workspaces](#module-4--launch-files--workspaces)
7. [Module 5 — ROS 2 Tools & Debugging](#module-5--ros-2-tools--debugging)
8. [Module 6 — TF2 & Transforms](#module-6--tf2--transforms)
9. [Module 7 — URDF & Robot Modeling](#module-7--urdf--robot-modeling)
10. [Module 8 — Gazebo Simulation](#module-8--gazebo-simulation)
11. [Module 9 — Xacro & Advanced URDF](#module-9--xacro--advanced-urdf)
12. [Module 10 — RViz & Visualization](#module-10--rviz--visualization)
13. [Module 11 — ROS 2 Actions](#module-11--ros-2-actions)
14. [Module 12 — Lifecycle Nodes](#module-12--lifecycle-nodes)
15. [Module 13 — Executors & Components](#module-13--executors--components)
16. [Final Project](#final-project)
17. [Best Practices Reference](#best-practices-reference)
18. [Quick Reference Cheat Sheet](#quick-reference-cheat-sheet)

---

## 1. Course Overview

### What You Will Build
By the end of this course you will have built a **complete simulated mobile robot** that:
- Moves autonomously using velocity commands
- Publishes sensor data (LiDAR, camera)
- Responds to service calls and action goals
- Is fully visualized in RViz
- Runs inside a custom Gazebo world

### Course Structure
| Module | Topic | Language | Estimated Time |
|--------|-------|----------|----------------|
| 1 | ROS 2 Fundamentals & Nodes | Python + C++ | 3 h |
| 2 | Topics & Communication | Python + C++ | 4 h |
| 3 | Services & Parameters | Python + C++ | 3 h |
| 4 | Launch Files & Workspaces | XML + Python | 3 h |
| 5 | ROS 2 Tools | CLI | 2 h |
| 6 | TF2 & Transforms | Python + C++ | 3 h |
| 7 | URDF & Robot Modeling | XML/URDF | 4 h |
| 8 | Gazebo Simulation | Gazebo | 4 h |
| 9 | Xacro & Advanced URDF | Xacro | 3 h |
| 10 | RViz & Visualization | RViz | 2 h |
| 11 | ROS 2 Actions | Python + C++ | 4 h |
| 12 | Lifecycle Nodes | Python + C++ | 3 h |
| 13 | Executors & Components | C++ | 3 h |
| Final | Complete Robot Project | Mixed | 5 h |

---

## 2. Environment Setup

### 2.1 Install Ubuntu 24.04
Use a native install, VM (VirtualBox/VMware), or WSL2 on Windows.

> Ubuntu 24.04 Noble Numbat is the required OS for ROS 2 Jazzy. Ubuntu 22.04 is **not** supported.

### 2.2 Install ROS 2 Jazzy

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Enable the Ubuntu Universe repository
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add ROS 2 apt repository
sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list

# Install ROS 2 Jazzy
sudo apt update && sudo apt upgrade
sudo apt install ros-jazzy-desktop
sudo apt install ros-dev-tools

# Source ROS 2 in every shell (add to ~/.bashrc)
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2.3 Verify Installation

```bash
ros2 doctor --report    # should report ROS_DISTRO=jazzy
ros2 run demo_nodes_py talker &
ros2 run demo_nodes_py listener
```

### 2.4 Install Gazebo Harmonic & Additional Packages

ROS 2 Jazzy officially pairs with **Gazebo Harmonic** (the new-generation Gazebo, formerly Ignition). It is a separate install from ROS 2.

```bash
# Install Gazebo Harmonic
sudo apt install ros-jazzy-ros-gz

# Additional ROS 2 tools
sudo apt install ros-jazzy-joint-state-publisher-gui
sudo apt install ros-jazzy-xacro
sudo apt install ros-jazzy-tf2-tools
sudo apt install ros-jazzy-tf-transformations
sudo apt install python3-colcon-common-extensions
sudo apt install python3-transforms3d        # needed for tf_transformations
```

> **Gazebo name change:** Starting from Gazebo Fortress the "Ignition" name was dropped. What was "Ignition Gazebo" is now just "Gazebo" (or "gz"). The classic `gazebo` simulator is end-of-life and is **not** used in this course.

### 2.5 Create Your Course Workspace

```bash
mkdir -p ~/ros2_course_ws/src
cd ~/ros2_course_ws
colcon build
source install/setup.bash
echo "source ~/ros2_course_ws/install/setup.bash" >> ~/.bashrc
```

---

## Module 1 — ROS 2 Fundamentals

### Learning Objectives
- Understand the ROS 2 architecture (DDS, nodes, graph)
- Create a ROS 2 package in Python and C++
- Write, build, and run your first node

### 1.1 What is ROS 2?

ROS 2 is a middleware framework for robot software. It provides:
- **Communication** between processes (nodes) via topics, services, and actions
- **Hardware abstraction** through a driver ecosystem
- **Tools** for visualization, debugging, and simulation
- **Build system** (colcon + ament) for managing packages

Key differences from ROS 1:
| ROS 1 | ROS 2 |
|-------|-------|
| roscore required | No master process |
| TCP/IP custom | DDS (Data Distribution Service) |
| Python 2/3 mixed | Python 3.12 (Jazzy) |
| No security | SROS2 security |
| Ubuntu 20.04 max | Ubuntu 24.04 (Jazzy) |

### 1.2 ROS 2 Graph Concepts

```
┌──────────┐   topic /cmd_vel   ┌──────────────┐
│ Teleop   ├──────────────────► │ Robot Driver │
│ Node     │                    │ Node         │
└──────────┘                    └──────┬───────┘
                                       │ topic /odom
                                       ▼
                                ┌──────────────┐
                                │ Nav Stack    │
                                │ Node         │
                                └──────────────┘
```

- **Node**: a single executable that performs one logical task
- **Topic**: named bus for streaming data (publisher → subscriber)
- **Service**: synchronous request/response call
- **Action**: asynchronous goal with feedback and result
- **Parameter**: configurable value stored per-node

### 1.3 Create a Python Package

```bash
cd ~/ros2_course_ws/src
ros2 pkg create --build-type ament_python my_py_pkg --dependencies rclpy
```

Package structure:
```
my_py_pkg/
├── my_py_pkg/
│   ├── __init__.py
│   └── my_first_node.py   ← your code goes here
├── package.xml
├── resource/
│   └── my_py_pkg
├── setup.cfg
└── setup.py
```

### 1.4 Your First Python Node

Create `my_py_pkg/my_py_pkg/my_first_node.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class MyFirstNode(Node):
    def __init__(self):
        super().__init__("my_first_node")
        self.get_logger().info("Hello from my first ROS 2 node!")
        self.counter = 0
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.counter += 1
        self.get_logger().info(f"Timer fired {self.counter} times")


def main(args=None):
    rclpy.init(args=args)
    node = MyFirstNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

Register the entry point in `setup.py`:

```python
entry_points={
    "console_scripts": [
        "my_first_node = my_py_pkg.my_first_node:main",
    ],
},
```

### 1.5 Your First C++ Package

```bash
cd ~/ros2_course_ws/src
ros2 pkg create --build-type ament_cmake my_cpp_pkg --dependencies rclcpp
```

Create `my_cpp_pkg/src/my_first_node.cpp`:

```cpp
#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"

class MyFirstNode : public rclcpp::Node {
public:
    MyFirstNode() : Node("my_first_node"), counter_(0) {
        RCLCPP_INFO(get_logger(), "Hello from my first C++ ROS 2 node!");
        timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MyFirstNode::timerCallback, this));
    }

private:
    void timerCallback() {
        counter_++;
        RCLCPP_INFO(get_logger(), "Timer fired %d times", counter_);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyFirstNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

Update `CMakeLists.txt`:

```cmake
add_executable(my_first_node src/my_first_node.cpp)
ament_target_dependencies(my_first_node rclcpp)
install(TARGETS my_first_node DESTINATION lib/${PROJECT_NAME})
```

### 1.6 Build and Run

```bash
cd ~/ros2_course_ws
colcon build --packages-select my_py_pkg my_cpp_pkg
source install/setup.bash

ros2 run my_py_pkg my_first_node
# In another terminal:
ros2 run my_cpp_pkg my_first_node
```

### Activity 1
> Create a node called `robot_news_station` that publishes a "news ticker" string every 0.5 seconds using a timer. The string should include the node name and a counter. Build and run it, verify the output with `ros2 node list` and `ros2 node info /robot_news_station`.

---

## Module 2 — Topics & Communication

### Learning Objectives
- Publish and subscribe to topics in Python and C++
- Use standard message types
- Inspect topics with CLI tools

### 2.1 Publisher/Subscriber Architecture

```
Publisher Node ──[/topic_name: MsgType]──► Subscriber Node
               ──[/topic_name: MsgType]──► Subscriber Node 2
```

- Many publishers → one topic → many subscribers
- Decoupled: publisher doesn't know who subscribes
- Asynchronous: non-blocking

### 2.2 Python Publisher

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RobotNewsStation(Node):
    def __init__(self):
        super().__init__("robot_news_station")
        self.pub_ = self.create_publisher(String, "/robot_news", 10)
        self.timer_ = self.create_timer(0.5, self.publish_news)
        self.get_logger().info("Robot News Station started")

    def publish_news(self):
        msg = String()
        msg.data = "Breaking news from " + self.get_name()
        self.pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotNewsStation()
    rclpy.spin(node)
    rclpy.shutdown()
```

### 2.3 Python Subscriber

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SmartphoneNode(Node):
    def __init__(self):
        super().__init__("smartphone")
        self.sub_ = self.create_subscription(
            String, "/robot_news", self.callback_robot_news, 10)

    def callback_robot_news(self, msg: String):
        self.get_logger().info(f"Received: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = SmartphoneNode()
    rclpy.spin(node)
    rclpy.shutdown()
```

Add both Python nodes to the existing `console_scripts` list in `setup.py`, and add `<depend>std_msgs</depend>` to `package.xml`:

```python
entry_points={
    "console_scripts": [
        "robot_news_station = my_py_pkg.robot_news_station:main",
        "smartphone = my_py_pkg.smartphone:main",
    ],
},
```

### 2.4 C++ Publisher

```cpp
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class RobotNewsStation : public rclcpp::Node {
public:
    RobotNewsStation() : Node("robot_news_station") {
        pub_ = create_publisher<std_msgs::msg::String>("/robot_news", 10);
        timer_ = create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&RobotNewsStation::publishNews, this));
    }

private:
    void publishNews() {
        auto msg = std_msgs::msg::String();
        msg.data = "News from " + std::string(get_name());
        pub_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotNewsStation>());
    rclcpp::shutdown();
}
```

### 2.5 C++ Subscriber

```cpp
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class SmartphoneNode : public rclcpp::Node {
public:
    SmartphoneNode() : Node("smartphone") {
        sub_ = create_subscription<std_msgs::msg::String>(
            "/robot_news", 10,
            std::bind(&SmartphoneNode::callbackRobotNews, this,
                      std::placeholders::_1));
    }

private:
    void callbackRobotNews(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "Received: %s", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SmartphoneNode>());
    rclcpp::shutdown();
    return 0;
}
```

For the C++ package, add `<depend>std_msgs</depend>` to `package.xml`, then add these lines to `CMakeLists.txt`:

```cmake
find_package(std_msgs REQUIRED)

add_executable(robot_news_station src/robot_news_station.cpp)
ament_target_dependencies(robot_news_station rclcpp std_msgs)

add_executable(smartphone src/smartphone.cpp)
ament_target_dependencies(smartphone rclcpp std_msgs)

install(TARGETS
  robot_news_station
  smartphone
  DESTINATION lib/${PROJECT_NAME})
```

### 2.6 Topic CLI Commands

```bash
ros2 topic list                          # list all active topics
ros2 topic info /robot_news              # publisher/subscriber count & type
ros2 topic echo /robot_news              # print messages in terminal
ros2 topic hz /robot_news                # measure publish rate
ros2 topic bw /robot_news                # measure bandwidth
ros2 topic pub /robot_news std_msgs/msg/String "data: 'hello'"
```

### 2.7 Common Message Types

| Package | Message | Use |
|---------|---------|-----|
| `std_msgs` | `String`, `Int32`, `Float64`, `Bool` | Simple data |
| `geometry_msgs` | `Twist`, `Pose`, `Point`, `Quaternion` | Robot motion |
| `sensor_msgs` | `LaserScan`, `Image`, `Imu`, `JointState` | Sensor data |
| `nav_msgs` | `Odometry`, `Path`, `OccupancyGrid` | Navigation |

### 2.8 Custom Message (Interface)

Create a package for interfaces:
```bash
ros2 pkg create my_robot_interfaces --build-type ament_cmake
mkdir -p my_robot_interfaces/msg
```

Create `msg/HardwareStatus.msg`:
```
int64 temperature
bool are_motors_ready
string debug_message
```

Update `CMakeLists.txt`:
```cmake
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HardwareStatus.msg"
)
```

Add to `package.xml`:
```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

### Activity 2
> Build a number publisher that sends integers from 1 to 100 on `/number` at 1 Hz, and a counter subscriber that accumulates the sum and publishes it on `/number_count`. Run both nodes and verify the count is increasing correctly.

---

## Module 3 — Services & Parameters

### Learning Objectives
- Write service servers and clients in Python and C++
- Declare, get, and set node parameters
- Use the parameter CLI

### 3.1 Services vs Topics

| | Topic | Service |
|-|-------|---------|
| Pattern | Publish / Subscribe | Request / Response |
| Direction | One-way | Two-way |
| Timing | Async, continuous | Sync, on-demand |
| Use case | Sensor streams, commands | Configuration, one-off actions |

### 3.2 Custom Service Definition

Create `srv/AddTwoInts.srv` in your interfaces package:
```bash
mkdir -p my_robot_interfaces/srv
```

```
int64 a
int64 b
---
int64 sum
```

Register it in `CMakeLists.txt`:
```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HardwareStatus.msg"
  "srv/AddTwoInts.srv"
)
```

### 3.3 Python Service Server

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import AddTwoInts


class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__("add_two_ints_server")
        self.server_ = self.create_service(
            AddTwoInts, "add_two_ints", self.callback_add_two_ints)
        self.get_logger().info("Add Two Ints server started")

    def callback_add_two_ints(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f"{request.a} + {request.b} = {response.sum}")
        return response


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(AddTwoIntsServer())
    rclpy.shutdown()
```

### 3.4 Python Service Client

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import AddTwoInts
from functools import partial


class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__("add_two_ints_client")
        self.call_add_two_ints(3, 4)

    def call_add_two_ints(self, a, b):
        client = self.create_client(AddTwoInts, "add_two_ints")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for server...")

        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call, a=a, b=b))

    def callback_call(self, future, a, b):
        response = future.result()
        self.get_logger().info(f"{a} + {b} = {response.sum}")


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(AddTwoIntsClient())
    rclpy.shutdown()
```

### 3.5 C++ Service Server

```cpp
#include <cinttypes>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/add_two_ints.hpp"

class AddTwoIntsServer : public rclcpp::Node {
public:
    AddTwoIntsServer() : Node("add_two_ints_server") {
        server_ = create_service<my_robot_interfaces::srv::AddTwoInts>(
            "add_two_ints",
            std::bind(&AddTwoIntsServer::callbackAddTwoInts, this,
                      std::placeholders::_1, std::placeholders::_2));
    }

private:
    void callbackAddTwoInts(
        const my_robot_interfaces::srv::AddTwoInts::Request::SharedPtr req,
        const my_robot_interfaces::srv::AddTwoInts::Response::SharedPtr res) {
        res->sum = req->a + req->b;
        RCLCPP_INFO(
            get_logger(), "%" PRId64 " + %" PRId64 " = %" PRId64,
            req->a, req->b, res->sum);
    }

    rclcpp::Service<my_robot_interfaces::srv::AddTwoInts>::SharedPtr server_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AddTwoIntsServer>());
    rclcpp::shutdown();
    return 0;
}
```

### 3.6 Service CLI Commands

```bash
ros2 service list                                    # list all services
ros2 service type /add_two_ints                      # get type
ros2 service call /add_two_ints my_robot_interfaces/srv/AddTwoInts "{a: 3, b: 4}"
```

### 3.7 Parameters

Parameters are named values stored per-node. Use them to make nodes configurable without recompilation.

**Python — declare and use parameters:**

```python
class MyNode(Node):
    def __init__(self):
        super().__init__("my_node")
        self.declare_parameter("robot_name", "my_robot")
        self.declare_parameter("move_speed", 1.5)
        name = self.get_parameter("robot_name").value
        speed = self.get_parameter("move_speed").value
        self.get_logger().info(f"Robot: {name}, Speed: {speed}")
```

**C++ — declare and use parameters:**

```cpp
MyNode() : Node("my_node") {
    declare_parameter("robot_name", "my_robot");
    declare_parameter("move_speed", 1.5);
    auto name = get_parameter("robot_name").as_string();
    auto speed = get_parameter("move_speed").as_double();
    RCLCPP_INFO(get_logger(), "Robot: %s, Speed: %.1f", name.c_str(), speed);
}
```

**Parameter CLI commands:**

```bash
ros2 param list /my_node
ros2 param get /my_node robot_name
ros2 param set /my_node robot_name "robo_007"
ros2 param dump /my_node                    # dump to YAML
ros2 param load /my_node params.yaml       # load from YAML
```

**Parameter file `params.yaml`:**

```yaml
my_node:
  ros__parameters:
    robot_name: "robo_007"
    move_speed: 2.0
```

Load at launch:
```bash
ros2 run my_pkg my_node --ros-args --params-file params.yaml
```

### Activity 3
> Create a `led_panel` service server that maintains a list of 3 LEDs (on/off). Accept a `SetLed` service call with `int64 led_number` and `bool state`, returning `bool success` and `string message`. Test it from the CLI.

---

## Module 4 — Launch Files & Workspaces

### Learning Objectives
- Write XML and Python launch files
- Pass arguments and remap topics in launch files
- Organize a multi-package workspace

### 4.1 Why Launch Files?

A launch file starts multiple nodes with a single command and lets you:
- Configure parameters inline
- Remap topic names
- Set namespace isolation
- Conditionally include other launch files

### 4.2 XML Launch File

Create `my_py_pkg/launch/my_first_launch.xml`:

```xml
<launch>
    <!-- Start the robot news station -->
    <node pkg="my_py_pkg" exec="robot_news_station" name="robot_news_station">
        <param name="robot_name" value="R2D2" />
    </node>

    <!-- Start the smartphone subscriber -->
    <node pkg="my_py_pkg" exec="smartphone" name="smartphone" />
</launch>
```

### 4.3 Python Launch File

Create `my_py_pkg/launch/my_first_launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value="my_robot",
        description="Name of the robot"
    )

    robot_news_station_node = Node(
        package="my_py_pkg",
        executable="robot_news_station",
        name="robot_news_station",
        parameters=[{"robot_name": LaunchConfiguration("robot_name")}],
        remappings=[("/robot_news", "/robot_news_v2")],
    )

    smartphone_node = Node(
        package="my_py_pkg",
        executable="smartphone",
        name="smartphone",
        remappings=[("/robot_news", "/robot_news_v2")],
    )

    return LaunchDescription([
        robot_name_arg,
        robot_news_station_node,
        smartphone_node,
    ])
```

### 4.4 Install Launch Files

Add to `setup.py` (Python packages):
```python
import os
from glob import glob

data_files=[
    ("share/ament_index/resource_index/packages",
     ["resource/" + package_name]),
    (os.path.join("share", package_name), ["package.xml"]),
    (os.path.join("share", package_name, "launch"),
     glob("launch/*.py") + glob("launch/*.xml")),
],
```

For CMake packages, add to `CMakeLists.txt`:
```cmake
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)
```

### 4.5 Run a Launch File

```bash
ros2 launch my_py_pkg my_first_launch.py
ros2 launch my_py_pkg my_first_launch.py robot_name:=R2D2

# XML
ros2 launch my_py_pkg my_first_launch.xml
```

### 4.6 Workspace Organization Best Practices

```
ros2_course_ws/
├── src/
│   ├── my_robot_interfaces/   ← all custom msg/srv/action definitions
│   ├── my_py_pkg/             ← Python nodes
│   ├── my_cpp_pkg/            ← C++ nodes
│   └── my_robot_bringup/      ← launch files and configs for the full system
├── install/
├── build/
└── log/
```

Rebuild only changed packages:
```bash
colcon build --packages-select my_py_pkg
colcon build --symlink-install   # Python: edits take effect without rebuild
```

### Activity 4
> Write a Python launch file that starts three nodes: `robot_news_station`, `smartphone`, and `add_two_ints_server`. Pass `robot_name` as a launch argument with default `"CourseBot"`. Run it and verify all three nodes appear in `ros2 node list`.

---

## Module 5 — ROS 2 Tools & Debugging

### Learning Objectives
- Use essential CLI and GUI tools
- Inspect the live node graph
- Record and replay data with rosbag2

### 5.1 Node Tools

```bash
ros2 node list                      # list running nodes
ros2 node info /my_node             # publishers, subscribers, services, params
```

### 5.2 Topic Tools

```bash
ros2 topic list -t                  # list with types
ros2 topic echo /chatter            # print messages
ros2 topic pub --once /chatter std_msgs/msg/String "data: 'hello'"
ros2 topic hz /chatter              # measure rate
ros2 topic bw /chatter              # measure bandwidth
ros2 interface show std_msgs/msg/String   # show message definition
```

### 5.3 Service Tools

```bash
ros2 service list -t
ros2 service call /add_two_ints \
    my_robot_interfaces/srv/AddTwoInts "{a: 5, b: 7}"
ros2 interface show my_robot_interfaces/srv/AddTwoInts
```

### 5.4 Parameter Tools

```bash
ros2 param list
ros2 param get /my_node my_param
ros2 param set /my_node my_param "new_value"
ros2 param dump /my_node > params.yaml
```

### 5.5 rqt — GUI Tool Suite

```bash
rqt                  # full GUI
rqt_graph            # node/topic graph visualization
rqt_plot             # plot numeric topics in real time
rqt_console          # filter and view log messages
```

### 5.6 rosbag2 — Record & Replay

```bash
# Record all topics
ros2 bag record -a -o my_recording

# Record specific topics
ros2 bag record /robot_news /odom -o my_recording

# Play back
ros2 bag play my_recording

# Inspect
ros2 bag info my_recording
```

### 5.7 Logging Levels

```python
self.get_logger().debug("Debug message")
self.get_logger().info("Info message")
self.get_logger().warn("Warning message")
self.get_logger().error("Error message")
self.get_logger().fatal("Fatal message")
```

Set level at runtime:
```bash
ros2 service call /my_node/set_logger_levels rcl_interfaces/srv/SetLoggerLevels \
    "{levels: [{name: 'my_node', level: 10}]}"
# 10=DEBUG 20=INFO 30=WARN 40=ERROR 50=FATAL
```

### Activity 5
> Start the publisher/subscriber nodes from Module 2. Use `ros2 topic hz`, `ros2 topic bw`, and `rqt_graph` to inspect the system. Record 10 seconds of data with rosbag2, then replay it and verify you see the same messages.

---

## Module 6 — TF2 & Transforms

### Learning Objectives
- Understand what TF is and why robots need it
- Broadcast and listen to static and dynamic transforms
- Visualize TF frames in RViz

### 6.1 What is TF2?

TF2 (Transform Library v2) tracks coordinate frames over time. For a robot, you need to know:
- Where is my base relative to the world? (`odom` → `base_link`)
- Where is my camera relative to my base? (`base_link` → `camera_link`)
- Where is an obstacle in the world frame?

```
world
  └── odom
        └── base_link
              ├── laser_link
              ├── camera_link
              └── left_wheel_link
                  right_wheel_link
```

### 6.2 Static Transform — Python

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import tf_transformations


class StaticFramePublisher(Node):
    def __init__(self):
        super().__init__("static_frame_publisher")
        self.broadcaster_ = StaticTransformBroadcaster(self)
        self.send_static_transform()

    def send_static_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "laser_link"
        t.transform.translation.x = 0.2
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.1
        q = tf_transformations.quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.broadcaster_.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = StaticFramePublisher()
    rclpy.spin(node)
    rclpy.shutdown()
```

### 6.3 Static Transform — CLI

```bash
# Publish a static transform from CLI (great for testing)
ros2 run tf2_ros static_transform_publisher \
    --x 0.2 --y 0.0 --z 0.1 \
    --roll 0 --pitch 0 --yaw 0 \
    --frame-id base_link --child-frame-id laser_link
```

### 6.4 Dynamic Transform Broadcaster

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import tf_transformations


class DynamicFramePublisher(Node):
    def __init__(self):
        super().__init__("dynamic_frame_publisher")
        self.broadcaster_ = TransformBroadcaster(self)
        self.timer_ = self.create_timer(0.1, self.broadcast_transform)
        self.angle_ = 0.0

    def broadcast_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = 1.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        self.angle_ += 0.01
        q = tf_transformations.quaternion_from_euler(0, 0, self.angle_)
        t.transform.rotation.x, t.transform.rotation.y = q[0], q[1]
        t.transform.rotation.z, t.transform.rotation.w = q[2], q[3]
        self.broadcaster_.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicFramePublisher()
    rclpy.spin(node)
    rclpy.shutdown()
```

### 6.5 TF Listener

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

class TFListenerNode(Node):
    def __init__(self):
        super().__init__("tf_listener")
        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)
        self.timer_ = self.create_timer(0.5, self.lookup_transform)

    def lookup_transform(self):
        try:
            t = self.tf_buffer_.lookup_transform(
                "base_link", "laser_link", rclpy.time.Time())
            self.get_logger().info(
                f"laser_link is at x={t.transform.translation.x:.2f} "
                f"relative to base_link")
        except Exception as e:
            self.get_logger().warn(str(e))


def main(args=None):
    rclpy.init(args=args)
    node = TFListenerNode()
    rclpy.spin(node)
    rclpy.shutdown()
```

### 6.6 TF CLI Tools

```bash
ros2 run tf2_tools view_frames            # generate frames.pdf
ros2 run tf2_ros tf2_echo base_link laser_link   # live transform output
```

### Activity 6
> Create a robot with three frames: `base_link`, `camera_link` (0.15 m in front, 0.2 m up), and `lidar_link` (0.0 m forward, 0.25 m up). Broadcast all transforms and verify the tree with `view_frames` and RViz.

---

## Module 7 — URDF & Robot Modeling

### Learning Objectives
- Understand what URDF is and how it represents a robot
- Write links with visual, collision, and inertia
- Write joints (fixed, revolute, continuous, prismatic)
- View your robot in RViz with the Robot State Publisher

### 7.1 What is URDF?

URDF (Unified Robot Description Format) is an XML format for describing robot geometry, kinematics, and dynamics.

```
robot
├── link: base_link
├── link: left_wheel
├── link: right_wheel
├── joint: base_left_wheel_joint  (connects base_link → left_wheel)
└── joint: base_right_wheel_joint (connects base_link → right_wheel)
```

### 7.2 Basic URDF Structure

Create `my_robot_description/urdf/my_robot.urdf`:

```xml
<?xml version="1.0"?>
<robot name="my_robot">

  <!-- ===== BASE LINK ===== -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <inertia ixx="0.0458" ixy="0" ixz="0"
               iyy="0.0875" iyz="0" izz="0.1208"/>
    </inertial>
  </link>

  <!-- ===== LEFT WHEEL ===== -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <origin xyz="0 0 0" rpy="1.5707 0 0"/>
      <material name="dark_grey">
        <color rgba="0.3 0.3 0.3 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <origin xyz="0 0 0" rpy="1.5707 0 0"/>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.00058" ixy="0" ixz="0"
               iyy="0.00058" iyz="0" izz="0.00125"/>
    </inertial>
  </link>

  <!-- ===== BASE → LEFT WHEEL JOINT ===== -->
  <joint name="base_left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="-0.15 0.225 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

</robot>
```

### 7.3 Joint Types

| Type | Description | Example |
|------|-------------|---------|
| `fixed` | No movement | Camera mount |
| `continuous` | Rotates freely (no limits) | Wheel |
| `revolute` | Rotates within limits | Robotic arm joint |
| `prismatic` | Slides linearly | Elevator lift |
| `floating` | 6 DOF | Free-floating body |

### 7.4 Computing Inertia Values

For a **box** (mass m, dimensions l×w×h):
```
ixx = m/12 * (h² + w²)
iyy = m/12 * (h² + l²)
izz = m/12 * (l² + w²)
```

For a **cylinder** (mass m, radius r, length l):
```
ixx = iyy = m/12 * (3r² + l²)
izz = m/2 * r²
```

### 7.5 Robot State Publisher

The `robot_state_publisher` node:
1. Reads the URDF from the `/robot_description` parameter
2. Listens to `/joint_states` for moving joints
3. Broadcasts the full TF tree

```python
# In a launch file:
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

robot_description = ParameterValue(
    Command(["xacro ", urdf_file_path]),
    value_type=str
)

robot_state_publisher_node = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    parameters=[{"robot_description": robot_description}]
)
```

### 7.6 Verify in RViz

```bash
ros2 launch my_robot_description display.launch.py
```

In RViz: add **RobotModel** display, set **Fixed Frame** to `base_link`, and add **TF** display.

### Activity 7
> Model a differential drive robot with: base box (0.5×0.3×0.15 m), two driven wheels (r=0.1 m), and one caster sphere (r=0.05 m) at the front. Add proper collision and inertia to all links. Visualize it in RViz.

---

## Module 8 — Gazebo Simulation

### Learning Objectives
- Launch a robot in Gazebo Harmonic (`gz sim`)
- Add Gz-compatible plugins to the URDF via `<gazebo>` tags
- Use the differential drive plugin for wheel control
- Bridge Gz topics to ROS 2 with `ros_gz_bridge`
- Spawn a robot in a custom world

### 8.1 Gazebo Harmonic Overview

ROS 2 Jazzy uses **Gazebo Harmonic** (the new-generation Gazebo). Key differences from the old "Gazebo Classic":

| Classic Gazebo | Gazebo Harmonic (gz) |
|----------------|----------------------|
| `gazebo` command | `gz sim` command |
| `libgazebo_ros_*.so` plugins | `gz-sim-*-system` plugins |
| Topics auto-bridged to ROS | Topics bridged via `ros_gz_bridge` |
| `gazebo_ros` spawn package | `ros_gz_sim` spawn package |
| SDF v1.7 | SDF v1.10 |

Gazebo Harmonic provides:
- Rigid body physics (DART, Bullet)
- Sensor simulation (LiDAR, camera, IMU, GPS)
- Plugin system for hardware emulation

### 8.2 Gz Plugins in the URDF

Gazebo Harmonic plugins are declared inside `<gazebo>` tags in the URDF. The plugin filenames follow the pattern `gz-sim-<name>-system`.

**Differential Drive Plugin:**

```xml
<gazebo>
  <plugin filename="gz-sim-diff-drive-system"
          name="gz::sim::systems::DiffDrive">
    <left_joint>base_left_wheel_joint</left_joint>
    <right_joint>base_right_wheel_joint</right_joint>
    <wheel_separation>0.45</wheel_separation>
    <wheel_radius>0.1</wheel_radius>
    <max_linear_acceleration>1.0</max_linear_acceleration>
    <max_angular_acceleration>2.0</max_angular_acceleration>
    <topic>cmd_vel</topic>
    <odom_topic>odom</odom_topic>
    <frame_id>odom</frame_id>
    <child_frame_id>base_link</child_frame_id>
    <odom_publish_frequency>30</odom_publish_frequency>
  </plugin>
</gazebo>
```

**Joint State Publisher Plugin** (needed to publish `/joint_states` from Gz):

```xml
<gazebo>
  <plugin filename="gz-sim-joint-state-publisher-system"
          name="gz::sim::systems::JointStatePublisher">
    <topic>joint_states</topic>
    <joint_name>base_left_wheel_joint</joint_name>
    <joint_name>base_right_wheel_joint</joint_name>
  </plugin>
</gazebo>
```

### 8.3 Add a LiDAR Sensor

URDF link and joint (same as before):

```xml
<link name="laser_link">
  <visual>
    <geometry><cylinder radius="0.05" length="0.04"/></geometry>
    <material name="black"><color rgba="0 0 0 1"/></material>
  </visual>
  <collision>
    <geometry><cylinder radius="0.05" length="0.04"/></geometry>
  </collision>
  <inertial>
    <mass value="0.1"/>
    <inertia ixx="0.000025" ixy="0" ixz="0"
             iyy="0.000025" iyz="0" izz="0.0000125"/>
  </inertial>
</link>

<joint name="base_laser_joint" type="fixed">
  <parent link="base_link"/>
  <child link="laser_link"/>
  <origin xyz="0.2 0 0.2" rpy="0 0 0"/>
</joint>
```

Gazebo Harmonic sensor definition (SDF v1.10 style inside `<gazebo reference>`):

```xml
<gazebo reference="laser_link">
  <sensor name="laser" type="gpu_lidar">
    <pose>0 0 0 0 0 0</pose>
    <topic>scan</topic>
    <update_rate>10</update_rate>
    <gz_frame_id>laser_link</gz_frame_id>
    <lidar>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.3</min>
        <max>12.0</max>
        <resolution>0.01</resolution>
      </range>
    </lidar>
    <visualize>true</visualize>
  </sensor>
</gazebo>

<!-- Enable sensor rendering -->
<gazebo>
  <plugin filename="gz-sim-sensors-system"
          name="gz::sim::systems::Sensors">
    <render_engine>ogre2</render_engine>
  </plugin>
</gazebo>
```

### 8.4 Create a Custom World

Create `worlds/my_world.sdf` (SDF v1.10 for Gazebo Harmonic):

```xml
<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="my_world">

    <!-- Physics -->
    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>

    <!-- Lighting -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Box obstacle -->
    <model name="box_obstacle">
      <static>true</static>
      <pose>2 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>0.5 0.5 1.0</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.5 0.5 1.0</size></box></geometry>
          <material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

### 8.5 Bridging Gz Topics to ROS 2

Gazebo Harmonic and ROS 2 run in **separate middleware domains**. The `ros_gz_bridge` node translates between them.

```bash
# Run a one-off bridge from the terminal
ros2 run ros_gz_bridge parameter_bridge \
    /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
    /odom@nav_msgs/msg/Odometry[gz.msgs.Odometry \
    /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
    /joint_states@sensor_msgs/msg/JointState[gz.msgs.Model
```

Bridge direction syntax: `@ROS_TYPE@gz.msgs.GzType` (bidirectional), `@ROS_TYPE[gz.msgs.GzType` (Gz→ROS only), `@ROS_TYPE]gz.msgs.GzType` (ROS→Gz only).

### 8.6 Launch File — Robot in Gazebo Harmonic

```python
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory("my_robot_description")
    xacro_file = os.path.join(pkg, "urdf", "my_robot.urdf.xacro")
    world_file = os.path.join(pkg, "worlds", "my_world.sdf")

    robot_description = ParameterValue(
        Command(["xacro ", xacro_file]), value_type=str)

    # Start Gazebo Harmonic
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"),
                         "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": f"-r {world_file}"}.items(),
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}],
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "my_robot", "-topic", "robot_description"],
        output="screen",
    )

    # Bridge Gz ↔ ROS 2 topics
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge,
    ])
```

### Activity 8
> Add a camera sensor (`type="camera"`) to your robot pointing forward. Extend the `ros_gz_bridge` to also bridge `/camera/image_raw`. Launch the robot in Gazebo Harmonic, drive it with `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"`, and verify the LiDAR scan and camera image appear in RViz.

---

## Module 9 — Xacro & Advanced URDF

### Learning Objectives
- Refactor URDF with Xacro properties and macros
- Use Xacro includes for modular robot descriptions
- Understand best practices for maintainable robot descriptions

### 9.1 Why Xacro?

Raw URDF repeats values and is hard to maintain. Xacro is a macro language that extends XML:
- **Properties** — named constants
- **Macros** — reusable blocks (like functions)
- **Math expressions** — `${pi/2}`, `${0.1 * 2}`
- **Conditionals** — `<xacro:if value="${use_sim}"/>`
- **Include** — split into multiple files

### 9.2 Rename URDF to Xacro

Rename `my_robot.urdf` → `my_robot.urdf.xacro` and add the XML namespace:

```xml
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
```

### 9.3 Properties

```xml
<xacro:property name="base_length" value="0.6"/>
<xacro:property name="base_width"  value="0.4"/>
<xacro:property name="base_height" value="0.2"/>
<xacro:property name="wheel_radius" value="0.1"/>
<xacro:property name="wheel_length" value="0.05"/>

<link name="base_link">
  <visual>
    <geometry>
      <box size="${base_length} ${base_width} ${base_height}"/>
    </geometry>
  </visual>
</link>
```

### 9.4 Math Expressions

```xml
<xacro:property name="pi" value="3.14159265"/>

<origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
<origin xyz="${-base_length/2 + 0.1} 0 0"/>
```

### 9.5 Macros — Reusable Links

```xml
<!-- Define the wheel macro -->
<xacro:macro name="wheel_link" params="name">
  <link name="${name}">
    <visual>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_length}"/>
      </geometry>
      <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
      <material name="dark_grey"><color rgba="0.3 0.3 0.3 1"/></material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_length}"/>
      </geometry>
      <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.00058" ixy="0" ixz="0"
               iyy="0.00058" iyz="0" izz="0.00125"/>
    </inertial>
  </link>
</xacro:macro>

<!-- Use the macro -->
<xacro:wheel_link name="left_wheel"/>
<xacro:wheel_link name="right_wheel"/>
```

### 9.6 Macros with Joint

```xml
<xacro:macro name="wheel_joint" params="name parent x_offset y_offset">
  <joint name="${name}_joint" type="continuous">
    <parent link="${parent}"/>
    <child link="${name}"/>
    <origin xyz="${x_offset} ${y_offset} 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
</xacro:macro>

<xacro:wheel_joint name="left_wheel"  parent="base_link"
                   x_offset="-0.15" y_offset="0.225"/>
<xacro:wheel_joint name="right_wheel" parent="base_link"
                   x_offset="-0.15" y_offset="-0.225"/>
```

### 9.7 Include Files

Split into multiple files for organization:
```xml
<!-- my_robot.urdf.xacro — main file -->
<xacro:include filename="$(find my_robot_description)/urdf/base.xacro"/>
<xacro:include filename="$(find my_robot_description)/urdf/wheels.xacro"/>
<xacro:include filename="$(find my_robot_description)/urdf/sensors.xacro"/>
<xacro:include filename="$(find my_robot_description)/urdf/gazebo.xacro"/>
```

### 9.8 Conditional Gazebo/Sim Tags

Use Xacro arguments to toggle Gz plugins so the same Xacro works with and without simulation:

```xml
<xacro:arg name="use_sim" default="false"/>
<xacro:property name="use_sim_" value="$(arg use_sim)"/>

<xacro:if value="${use_sim_}">
  <!-- Only included when use_sim:=true -->
  <gazebo>
    <plugin filename="gz-sim-diff-drive-system"
            name="gz::sim::systems::DiffDrive">
      <left_joint>base_left_wheel_joint</left_joint>
      <right_joint>base_right_wheel_joint</right_joint>
      <wheel_separation>${base_width + wheel_length}</wheel_separation>
      <wheel_radius>${wheel_radius}</wheel_radius>
      <topic>cmd_vel</topic>
    </plugin>
  </gazebo>
</xacro:if>
```

Pass the argument when processing:
```bash
xacro my_robot.urdf.xacro use_sim:=true > my_robot_sim.urdf
```

Process Xacro file:
```bash
xacro my_robot.urdf.xacro > my_robot.urdf          # one-shot
xacro my_robot.urdf.xacro use_sim:=true > out.urdf  # with args
```

### Activity 9
> Refactor your Module 7 URDF into Xacro. Use properties for all dimensions, a `wheel` macro to avoid repetition, and separate files for base/wheels/sensors/gazebo. Verify the generated URDF is identical to the original.

---

## Module 10 — RViz & Visualization

### Learning Objectives
- Navigate the RViz GUI
- Add and configure displays
- Save and reload RViz configurations in launch files
- Use interactive markers

### 10.1 Launching RViz

```bash
rviz2                                   # bare RViz
rviz2 -d my_config.rviz                 # load a saved config
```

### 10.2 Common Displays

| Display | Topic | Use |
|---------|-------|-----|
| RobotModel | `/robot_description` | Show URDF |
| TF | — | Show all frames |
| LaserScan | `/scan` | Show LiDAR hits |
| PointCloud2 | `/points` | Show 3D point cloud |
| Image | `/camera/image_raw` | Camera feed |
| Odometry | `/odom` | Show pose with covariance |
| Path | `/path` | Show planned/executed path |
| Map | `/map` | Show occupancy grid |
| Marker | `/visualization_marker` | Custom shapes |

### 10.3 Save RViz Config in Launch File

```python
import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory("my_robot_bringup")
    rviz_config = os.path.join(pkg, "rviz", "robot.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription([rviz_node])
```

Install the rviz directory:
```cmake
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/)
```

### 10.4 Visualization Markers (Python)

Add `visualization_msgs` to your package dependencies before using markers.

```python
from visualization_msgs.msg import Marker

def publish_sphere_marker(self, x, y, z):
    m = Marker()
    m.header.frame_id = "base_link"
    m.header.stamp = self.get_clock().now().to_msg()
    m.ns = "obstacles"
    m.id = 0
    m.type = Marker.SPHERE
    m.action = Marker.ADD
    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.position.z = z
    m.scale.x = m.scale.y = m.scale.z = 0.3
    m.color.r = 1.0
    m.color.g = 0.0
    m.color.b = 0.0
    m.color.a = 1.0
    self.marker_pub_.publish(m)
```

### Activity 10
> Configure an RViz session showing RobotModel, TF, LaserScan, and Odometry for your Gazebo robot. Save the config to `my_robot_bringup/rviz/robot.rviz` and load it automatically from your launch file.

---

## Module 11 — ROS 2 Actions

### Learning Objectives
- Understand the action pattern and when to use it
- Define a custom action
- Write an action server and client in Python and C++
- Implement goal policies (accept/reject, cancel)

### 11.1 Actions vs Services

| | Service | Action |
|-|---------|--------|
| Pattern | Request/Response | Goal/Feedback/Result |
| Duration | Instant | Long-running |
| Feedback | None | Continuous |
| Cancel | No | Yes |
| Use case | Get sensor value | Navigate to goal |

### 11.2 Action Definition

Create `action/NavigateToPoint.action` in your interfaces package:
```bash
mkdir -p my_robot_interfaces/action
```

```
# Goal
float64 x
float64 y
float64 tolerance
---
# Result
bool success
string message
float64 final_distance
---
# Feedback
float64 current_distance
float64 estimated_time_remaining
```

Update `CMakeLists.txt`:
```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HardwareStatus.msg"
  "srv/AddTwoInts.srv"
  "action/NavigateToPoint.action"
)
```

### 11.3 Python Action Server

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from my_robot_interfaces.action import NavigateToPoint
import math
import time


class NavigateToPointServer(Node):
    def __init__(self):
        super().__init__("navigate_to_point_server")
        self.action_server_ = ActionServer(
            self,
            NavigateToPoint,
            "navigate_to_point",
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
        )
        self.current_x_ = 0.0
        self.current_y_ = 0.0

    def goal_callback(self, goal_request):
        self.get_logger().info(
            f"Received goal: ({goal_request.x}, {goal_request.y})")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Goal cancel requested")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info("Executing goal...")
        goal = goal_handle.request
        feedback_msg = NavigateToPoint.Feedback()

        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = NavigateToPoint.Result()
                result.success = False
                result.message = "Goal cancelled"
                return result

            dx = goal.x - self.current_x_
            dy = goal.y - self.current_y_
            dist = math.sqrt(dx**2 + dy**2)

            if dist <= goal.tolerance:
                break

            # Move towards goal (simplified)
            step = min(0.1, dist)
            self.current_x_ += step * dx / dist
            self.current_y_ += step * dy / dist

            feedback_msg.current_distance = dist
            feedback_msg.estimated_time_remaining = dist / 0.1
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)

        goal_handle.succeed()
        result = NavigateToPoint.Result()
        result.success = True
        result.message = "Reached goal"
        result.final_distance = math.sqrt(
            (goal.x - self.current_x_)**2 + (goal.y - self.current_y_)**2)
        return result


def main(args=None):
    rclpy.init(args=args)
    node = NavigateToPointServer()
    rclpy.spin(node)
    rclpy.shutdown()
```

### 11.4 Python Action Client

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from my_robot_interfaces.action import NavigateToPoint


class NavigateToPointClient(Node):
    def __init__(self):
        super().__init__("navigate_to_point_client")
        self.client_ = ActionClient(self, NavigateToPoint, "navigate_to_point")

    def send_goal(self, x, y, tolerance=0.1):
        self.client_.wait_for_server()
        goal = NavigateToPoint.Goal()
        goal.x = x
        goal.y = y
        goal.tolerance = tolerance

        self.get_logger().info(f"Sending goal: ({x}, {y})")
        future = self.client_.send_goal_async(
            goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("Goal rejected!")
            return
        self.get_logger().info("Goal accepted")
        result_future = handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"Distance remaining: {fb.current_distance:.2f} m")

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(
            f"Result: success={result.success}, message='{result.message}'")


def main(args=None):
    rclpy.init(args=args)
    node = NavigateToPointClient()
    node.send_goal(3.0, 2.0)
    rclpy.spin(node)
    rclpy.shutdown()
```

### 11.5 Action CLI Commands

```bash
ros2 action list
ros2 action info /navigate_to_point
ros2 action send_goal /navigate_to_point \
    my_robot_interfaces/action/NavigateToPoint \
    "{x: 3.0, y: 2.0, tolerance: 0.1}"

# With feedback:
ros2 action send_goal --feedback /navigate_to_point \
    my_robot_interfaces/action/NavigateToPoint \
    "{x: 3.0, y: 2.0, tolerance: 0.1}"
```

### Activity 11
> Implement a `CountUntil` action: goal has `int64 count_until` and `float64 period`, feedback has `int64 current_count`, result has `int64 reached_count`. The server counts up every `period` seconds, and supports cancellation. Write both Python and C++ versions.

---

## Module 12 — Lifecycle Nodes

### Learning Objectives
- Understand the lifecycle node state machine
- Implement lifecycle callbacks
- Create an initialization sequence using lifecycle nodes
- Manage nodes with the lifecycle CLI

### 12.1 Why Lifecycle Nodes?

Standard nodes start all their work in `__init__`. Lifecycle nodes have explicit states so you can:
- **Configure** hardware/resources before activating
- **Activate** only when everything is ready
- **Deactivate** cleanly without destroying the node
- **Shutdown** with proper cleanup

### 12.2 Lifecycle State Machine

```
Unconfigured
     │  configure()
     ▼
  Inactive
     │  activate()          │  cleanup()
     ▼                      │
  Active ──────────────────►│
     │  deactivate()        │
     ▼                      │
  Inactive ─────────────────┘
     │  shutdown()
     ▼
 Finalized
```

### 12.3 Python Lifecycle Node

```python
#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from std_msgs.msg import String


class CameraDriverNode(LifecycleNode):
    def __init__(self):
        super().__init__("camera_driver")

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Configuring camera...")
        self.pub_ = self.create_lifecycle_publisher(String, "/camera/status", 10)
        self.timer_ = self.create_timer(1.0, self.publish_status)
        self.timer_.cancel()        # timer inactive until activated
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating camera...")
        self.timer_.reset()         # start publishing
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating camera...")
        self.timer_.cancel()
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up camera...")
        self.destroy_timer(self.timer_)
        self.destroy_publisher(self.pub_)
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Shutting down camera...")
        return TransitionCallbackReturn.SUCCESS

    def publish_status(self):
        msg = String()
        msg.data = "Camera OK"
        self.pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraDriverNode()
    rclpy.spin(node)
    rclpy.shutdown()
```

### 12.4 Lifecycle CLI

```bash
ros2 lifecycle list /camera_driver           # list available transitions
ros2 lifecycle get /camera_driver            # get current state
ros2 lifecycle set /camera_driver configure
ros2 lifecycle set /camera_driver activate
ros2 lifecycle set /camera_driver deactivate
ros2 lifecycle set /camera_driver cleanup
ros2 lifecycle set /camera_driver shutdown
```

### 12.5 Lifecycle Manager (Auto-configure)

Use the `nav2_lifecycle_manager` or write your own:

```python
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

class LifecycleManager(Node):
    def __init__(self):
        super().__init__("lifecycle_manager")
        self.change_state_client_ = self.create_client(
            ChangeState, "/camera_driver/change_state")
        self.startup_sequence()

    def change_state(self, transition_id):
        req = ChangeState.Request()
        req.transition.id = transition_id
        self.change_state_client_.call_async(req)

    def startup_sequence(self):
        self.change_state(Transition.TRANSITION_CONFIGURE)
        # wait, then:
        self.change_state(Transition.TRANSITION_ACTIVATE)
```

### Activity 12
> Convert the `robot_news_station` node from Module 2 into a lifecycle node. In `on_configure` create the publisher, in `on_activate` start the timer, in `on_deactivate` stop the timer. Test the full configure → activate → deactivate → cleanup sequence from the CLI.

---

## Module 13 — Executors & Components

### Learning Objectives
- Understand how `rclpy.spin` works internally
- Use single-threaded and multi-threaded executors
- Compose multiple nodes into one executable
- Create a ROS 2 component loadable at runtime

### 13.1 How spin() Works

`rclpy.spin(node)` is equivalent to:

```python
executor = SingleThreadedExecutor()
executor.add_node(node)
executor.spin()
```

The executor's event loop checks for pending callbacks (timer, topic, service, action) and calls them one at a time.

### 13.2 Single-Threaded vs Multi-Threaded

| Executor | Callbacks | Use When |
|----------|-----------|----------|
| `SingleThreadedExecutor` | Sequential | No blocking callbacks |
| `MultiThreadedExecutor` | Parallel | Long-running callbacks |

### 13.3 Multi-Threaded Executor

```python
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

class MyNode(Node):
    def __init__(self):
        super().__init__("my_node")
        # This group allows callbacks to run in parallel
        reentrant_group = ReentrantCallbackGroup()
        # This group ensures mutual exclusion
        exclusive_group = MutuallyExclusiveCallbackGroup()

        self.sub_ = self.create_subscription(
            String, "/topic", self.callback, 10,
            callback_group=reentrant_group)
        self.timer_ = self.create_timer(
            1.0, self.timer_callback,
            callback_group=exclusive_group)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()
```

### 13.4 Multiple Nodes in One Executable

```python
def main(args=None):
    rclpy.init(args=args)

    node1 = RobotNewsStation()
    node2 = SmartphoneNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node1)
    executor.add_node(node2)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        rclpy.shutdown()
```

### 13.5 ROS 2 Components (C++)

Components allow loading nodes into a container at runtime without recompiling.

Create `src/my_component.cpp`:

```cpp
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"

namespace my_cpp_pkg {

class MyComponent : public rclcpp::Node {
public:
    explicit MyComponent(const rclcpp::NodeOptions& options)
        : Node("my_component", options) {
        pub_ = create_publisher<std_msgs::msg::String>("/output", 10);
        timer_ = create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                auto msg = std_msgs::msg::String();
                msg.data = "From component";
                pub_->publish(msg);
            });
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace my_cpp_pkg

RCLCPP_COMPONENTS_REGISTER_NODE(my_cpp_pkg::MyComponent)
```

`CMakeLists.txt`:
```cmake
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

add_library(my_component SHARED src/my_component.cpp)
ament_target_dependencies(my_component rclcpp rclcpp_components std_msgs)
rclcpp_components_register_nodes(my_component "my_cpp_pkg::MyComponent")

install(TARGETS my_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
```

Add matching dependencies to `package.xml`:
```xml
<depend>rclcpp_components</depend>
<depend>std_msgs</depend>
```

Load at runtime:
```bash
ros2 run rclcpp_components component_container &
ros2 component load /ComponentManager my_cpp_pkg my_cpp_pkg::MyComponent
ros2 component list
ros2 component unload /ComponentManager 1
```

Load in launch file:
```python
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

container = ComposableNodeContainer(
    name="my_container",
    namespace="",
    package="rclcpp_components",
    executable="component_container",
    composable_node_descriptions=[
        ComposableNode(
            package="my_cpp_pkg",
            plugin="my_cpp_pkg::MyComponent",
            name="my_component",
        ),
    ],
)
```

### Activity 13
> Write two C++ components: `NumberPublisher` (publishes an integer every second) and `NumberCounter` (subscribes and accumulates). Load both into a single component container using a launch file. Verify only one process is running with `ps aux | grep ros2`.

---

## Final Project

### The Course Robot: DiffBot

Build a complete simulated differential drive robot from scratch that:

1. **URDF/Xacro** — full robot description with base, wheels, caster, LiDAR, camera
2. **Gazebo** — spawns in a custom world with obstacles, physics & sensors
3. **TF tree** — correct `odom` → `base_link` → sensor frames
4. **Navigation** — responds to `/cmd_vel`, publishes `/odom`
5. **Action server** — `NavigateToPoint` driving the robot to 2D goals
6. **Lifecycle** — sensor nodes managed through configure/activate lifecycle
7. **Launch** — single launch file brings up everything (Gazebo + RViz + all nodes)
8. **Parameters** — robot dimensions and speeds configurable via YAML

### Project Structure

```
ros2_course_ws/src/
├── my_robot_interfaces/          # msg, srv, action definitions
├── my_robot_description/         # URDF/Xacro, worlds, meshes
│   ├── urdf/
│   │   ├── my_robot.urdf.xacro
│   │   ├── base.xacro
│   │   ├── wheels.xacro
│   │   ├── sensors.xacro
│   │   └── gazebo.xacro
│   └── worlds/
│       └── course_world.sdf
├── my_robot_navigation/          # action server, velocity controller
└── my_robot_bringup/             # launch files, configs, rviz
    ├── launch/
    │   ├── robot_sim.launch.py
    │   └── robot_real.launch.py
    ├── rviz/
    │   └── robot.rviz
    └── config/
        └── robot_params.yaml
```

### Project Checklist

- [ ] Robot spawns in Gazebo Harmonic (`gz sim`) without falling through the ground
- [ ] LiDAR scan visible in RViz (bridged via `ros_gz_bridge`)
- [ ] Camera image stream active on `/camera/image_raw`
- [ ] `ros2 topic pub /cmd_vel` drives the robot
- [ ] `/odom` updates while robot moves
- [ ] TF tree is complete (`view_frames` shows no broken links)
- [ ] `NavigateToPoint` action server accepts, executes with feedback, and returns result
- [ ] Lifecycle camera driver goes through full state machine
- [ ] `use_sim_time: true` set for all nodes when running in simulation
- [ ] Single launch file starts the complete system (Gazebo + bridge + RSP + RViz)

---

## Best Practices Reference

### Naming Conventions

```
Node names:    snake_case       (robot_state_publisher)
Topic names:   /snake_case      (/joint_states)
Service names: /snake_case      (/set_led_state)
Action names:  /snake_case      (/navigate_to_point)
Package names: snake_case       (my_robot_description)
Classes:       PascalCase       (RobotStatePublisher)
```

### Node Design Principles

1. **One node = one responsibility** — don't bundle unrelated logic
2. **Parameterize** magic numbers — never hardcode frame names, topic names, or rates
3. **Use QoS profiles** — choose reliability/durability appropriate to sensor vs command data
4. **Handle missing dependencies** — check `wait_for_service` before calling
5. **Log at the right level** — `debug` for per-cycle data, `info` for state changes
6. **Avoid blocking in callbacks** — keep callbacks fast; use actions for long tasks

### Interface Design Principles

1. **Messages** — use standard types when they fit (`geometry_msgs/Twist`)
2. **Custom messages** — only when standard types don't fit your needs
3. **Services** — short, synchronous, guaranteed-response operations
4. **Actions** — anything that takes more than ~1 second or needs feedback

### Build & Package Hygiene

```bash
# Always build only what changed
colcon build --packages-select my_pkg

# Symlink install for Python development
colcon build --symlink-install

# Check interface generation
ros2 interface list | grep my_robot

# Verify no missing dependencies
rosdep install --from-paths src --ignore-src -r -y
```

---

## Quick Reference Cheat Sheet

### Node Lifecycle

```bash
ros2 run <pkg> <exec>                           # run a node
ros2 run <pkg> <exec> --ros-args -r /old:=/new  # remap
ros2 run <pkg> <exec> --ros-args -p name:=value # set param
ros2 run <pkg> <exec> --ros-args --params-file f.yaml
ros2 launch <pkg> <launch_file> key:=value
```

### Introspection

```bash
ros2 node list / info <name>
ros2 topic list / echo / hz / pub
ros2 service list / call
ros2 action list / send_goal
ros2 param list / get / set / dump
ros2 interface show <type>
ros2 bag record / play / info
rqt_graph
```

### TF

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo <parent> <child>
ros2 run tf2_ros static_transform_publisher ...
```

### Lifecycle

```bash
ros2 lifecycle get / list / set <node> <transition>
```

### Components

```bash
ros2 component list
ros2 component load /Container <pkg> <plugin>
ros2 component unload /Container <id>
```

### Gazebo Harmonic (gz)

```bash
gz sim                          # open Gazebo with empty world
gz sim -r my_world.sdf          # open and run a world
gz sim --help

gz topic list                   # list Gz topics (separate from ROS)
gz topic echo /scan             # echo a Gz topic

# Bridge a topic between Gz and ROS 2
ros2 run ros_gz_bridge parameter_bridge \
    /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan

# Spawn model into running Gz session
ros2 run ros_gz_sim create -name my_robot -topic robot_description
```

---

## Resources

| Resource | Link |
|----------|------|
| ROS 2 Jazzy Docs | https://docs.ros.org/en/jazzy |
| ROS 2 Jazzy Tutorials | https://docs.ros.org/en/jazzy/Tutorials.html |
| Gazebo Harmonic Docs | https://gazebosim.org/docs/harmonic |
| ros_gz Bridge Guide | https://github.com/gazebosim/ros_gz/tree/jazzy |
| ROS 2 Design | https://design.ros2.org |
| ROS Discourse | https://discourse.ros.org |
| Robotics Stack Exchange | https://robotics.stackexchange.com |
| URDF Tutorials | https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF |

---

*Course version 1.0 — ROS 2 Jazzy Jalisco / Gazebo Harmonic*
