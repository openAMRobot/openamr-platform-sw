# openamrobot_bringup

ROS 2 package for top-level robot and simulation launch composition.

This package contains launch files that start multiple OpenAMRobot subsystems together. The default profile for anything with camera/docking is the composed pipeline `bringup_composed.launch.py`; `bringup.launch.py` (non-composed) is for nav-only debug with `use_camera:=false`.

Expected future responsibilities:

- robot bringup launch files
- simulation bringup launch files
- real robot bringup launch files
- launch composition for description, simulation, navigation, docking, control, drivers, and perception
- high-level startup workflows

This package should not contain low-level robot models, Nav2 parameters, docking algorithms, drivers, or perception logic.

Status: active (composed and non-composed bring-up launch files in use).