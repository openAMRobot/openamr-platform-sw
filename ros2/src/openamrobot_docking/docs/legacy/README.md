# Docking documentation

This folder documents how the current AprilTag-based docking pipeline is configured for the OpenAMRobot docking package. It is intended to help contributors and student interns understand the existing system, reconfigure it carefully, and reproduce results.

Read in this order:
1. `02_apriltag_ros.md` - How the AprilTag pipeline is launched, topics/remaps, and how to change tag IDs, sizes, and frames.
2. `03_tf_frames.md` - Required TF chain and example static transforms for `base_link`, `camera_link`, and `camera_optical_frame`.
3. `04_docking_pipeline.md` - End-to-end docking flow, nodes involved, and how detections reach `opennav_docking`.
4. `05_dock_setup_map.md` - How to place/measure the dock (tag) pose in the `map` frame and set `home_dock.pose`.
5. `06_parameters.md` - Quick reference for key parameters across AprilTag, pose publisher, trigger, and docking server.
6. `07_reproduce_results.md` - Step-by-step checklist to reproduce results from a clean start.
7. `08_troubleshooting.md` - Common failure modes and what to check first.
8. `09_diagrams.md` - Mermaid diagrams: block diagram, TF tree, state flow, and parameter dependencies.

If you add new tags, change frame names, or alter docking parameters, update the relevant doc section so the workflow stays reproducible.
