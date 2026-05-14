# Camera Setup, Calibration, and AprilTag Detection

This guide documents the full procedure to go from a fresh ROS2 Jazzy install to a working AprilTag detection pipeline. It covers every problem encountered during the initial setup and how to fix them, so you can reproduce results without losing time on the same issues.

---

## Environment

| Component | Version |
|---|---|
| OS | Ubuntu 24.04 (Noble) |
| ROS2 | Jazzy |
| Camera | USB webcam (UVC-compatible) |
| AprilTag family | 36h11 |

---

## 1. Install dependencies

All required packages are available in the Jazzy apt repository:

```bash
sudo apt install \
  ros-jazzy-opennav-docking \
  ros-jazzy-opennav-docking-msgs \
  ros-jazzy-apriltag-ros \
  ros-jazzy-camera-ros \
  ros-jazzy-image-proc \
  ros-jazzy-nav2-lifecycle-manager \
  ros-jazzy-nav2-bringup \
  ros-jazzy-camera-calibration \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-tools
```

---

## 2. Build the package

```bash
cd /path/to/openamrobot-docking-main   # adapt to your actual path
source /opt/ros/jazzy/setup.bash
colcon build --packages-select openamrobot_docking
source install/setup.bash
```

> **Every time you open a new terminal**, you need to run:
> ```bash
> source /opt/ros/jazzy/setup.bash
> source /path/to/openamrobot-docking-main/install/setup.bash
> ```

---

## 3. Print the calibration targets

Two files are included in the package root:

| File | Purpose |
|---|---|
| `damier.png` | Checkerboard for camera calibration |
| `tag0_big.png` | AprilTag ID 0 (family 36h11) for the docking station |

**Print both at 100% scale — do not let the printer rescale them.**

After printing, take two measurements with a ruler:

1. **One square of `damier.png`** — measure the side of one square in meters. You will pass this to the calibration tool.
2. **The black border edge of `tag0_big.png`** — measure the outer edge of the black square in meters. You will put this in `config/tags_36h11.yaml`.

> **Example (our setup):** the printed tag measured 5.55 cm → `size: 0.0555` in the config.  
> Your printed size may differ depending on your printer and paper settings.

**Glue `damier.png` onto a rigid flat surface** (cardboard, wood, etc.). Any warping will prevent corner detection and make calibration impossible.

---

## 4. Camera calibration

### 4.1 Find your camera topics

Start the camera node:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run camera_ros camera_node
```

In a second terminal, list the published topics to find the image topic name:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list | grep image
```

You will see something like:

```
/camera/image_raw
/camera/image_raw/compressed
```

> **The topic you need** is the one of type `sensor_msgs/msg/Image`. Confirm with:
> ```bash
> ros2 topic info /camera/image_raw
> ```
> **Example (our setup):** the image topic was `/camera/image_raw` and the camera namespace was `/camera`.  
> Yours may differ — adapt the remap in step 4.3 accordingly.

### 4.2 Find the expected calibration filename

Look at the camera node output in the first terminal. You will see a line like:

```
[INFO] camera calibration URL: file:///home/<user>/.ros/camera_info/<camera_name>.yaml
[WARN] Camera calibration file ... not found
```

**Copy the full filename** (everything after `camera_info/`). You will need it in steps 4.4 and 4.5.

> **Example (our setup):**
> ```
> USB2_0HDUVCWebCam_USB2_0HD___SB__PC00_XHCI_RHUB_HS07_7_1_0_322e_202c_1280x720.yaml
> ```
> Your filename will be different — it encodes your specific camera model and resolution.

### 4.3 Run the calibration tool

Open a **second terminal** and run:

```bash
source /opt/ros/jazzy/setup.bash
export QT_QPA_PLATFORM=xcb
ros2 run camera_calibration cameracalibrator \
  --size <COLS>x<ROWS> \
  --square <SQUARE_SIZE_METERS> \
  --ros-args \
  --remap image:=<YOUR_IMAGE_TOPIC> \
  --remap camera:=<YOUR_CAMERA_NAMESPACE>
```

**Adapt these values to your setup:**

| Parameter | How to get it | Example (our setup) |
|---|---|---|
| `--size` | Count the **inner corners** of your checkerboard (where 4 squares meet), not the squares | `9x6` (for a 10×7 square board) |
| `--square` | Measure one square with a ruler (in meters) | `0.022` (2.2 cm) |
| `--remap image` | The image topic found in step 4.1 | `/camera/image_raw` |
| `--remap camera` | The camera namespace (everything before `/image_raw`) | `/camera` |

> **How to count inner corners:** a board with 10 columns × 7 rows of squares has (10−1) × (7−1) = **9×6 inner corners**.

A window will open showing the camera feed with colored lines overlaid when the checkerboard is detected.

### 4.4 Collect calibration samples

Move the checkerboard slowly in front of the camera until all four progress bars turn green:

| Bar | What it measures |
|---|---|
| X | Left ↔ Right coverage |
| Y | Up ↔ Down coverage |
| Size | Near ↔ Far coverage |
| Skew | Tilt coverage |

Tips:
- Hold the board still for a moment at each position
- Cover the full field of view: corners, center, close, far, tilted
- Make sure the board is well lit with no reflections or shadows

When all bars are green, click **Calibrate** (takes a few seconds), then **Save**.

The calibration data is saved to `/tmp/calibrationdata.tar.gz`.

### 4.5 Install the calibration file

```bash
cd /tmp && tar -xzf calibrationdata.tar.gz
mkdir -p ~/.ros/camera_info
cp ost.yaml ~/.ros/camera_info/<camera_name>.yaml
```

Replace `<camera_name>` with the exact filename you copied in step 4.2.

### 4.6 Fix the camera name inside the YAML file

The calibration tool always saves `camera_name: narrow_stereo` regardless of your actual camera. The camera driver checks this field and will warn if it does not match.

Open the file:

```bash
nano ~/.ros/camera_info/<camera_name>.yaml
```

Change:
```yaml
camera_name: narrow_stereo
```
To the filename without the `.yaml` extension:
```yaml
camera_name: <camera_name_without_extension>
```

> **Example (our setup):** the value became:
> ```yaml
> camera_name: USB2_0HDUVCWebCam_USB2_0HD___SB__PC00_XHCI_RHUB_HS07_7_1_0_322e_202c_1280x720
> ```

### 4.7 Verify calibration loaded correctly

Stop the camera node (Ctrl+C) and restart it:

```bash
ros2 run camera_ros camera_node
```

You should now see the calibration URL line **without any error or warning** about calibration. The camera is ready.

---

## 5. AprilTag detection

### 5.1 Update the tag configuration

Edit `config/tags_36h11.yaml` to match your printed tag:

```yaml
tag:
    ids: [0]                        # ID of your printed tag
    frames: [charging_dock_apriltag]  # TF frame name — keep this unless you changed it
```

Update `size` with your measured tag edge length:

```yaml
size: 0.0555   # replace with your measurement in meters
```

### 5.2 Launch the AprilTag pipeline

**Stop any standalone camera node** before this step. The launch file starts its own camera instance — running two processes that try to open the same camera will fail.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch openamrobot_docking apriltag.launch.yml
```

This starts three nodes in one component container:
1. `camera_ros` — camera driver
2. `image_proc::RectifyNode` — undistorts the image using the calibration
3. `AprilTagNode` — detects tags and publishes TF

### 5.3 Verify tag detection

In a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /tf
```

Hold the printed tag in front of the camera. When detected, you will see a TF published:

```yaml
transforms:
- header:
    frame_id: camera
  child_frame_id: charging_dock_apriltag
  transform:
    translation:
      x: ...
      y: ...
      z: ...   # distance to tag in meters
```

When the tag leaves the field of view, `transforms: []` is published — this is normal.

---

## 6. Problems encountered and fixes

### Camera is uncalibrated
**Symptom:**
```
[ERROR] Rectified topic '/camera/image_rect' requested but camera publishing
'/.../camera_info' is uncalibrated
```
**Cause:** No calibration file exists. The camera publishes a `camera_info` with a zero matrix.  
**Fix:** Follow section 4 to calibrate and install the calibration file.

---

### Calibration tool cannot receive images
**Symptom:**
```
[WARN] No publishers available for topic /camera/camera/image_raw.
```
**Cause:** Wrong topic remap — the topic name used in the remap does not match what the camera actually publishes.  
**Fix:** Always check what your camera publishes with `ros2 topic list | grep image` before running the calibration tool, and adapt the `--remap image:=` accordingly.

---

### Qt platform plugin error
**Symptom:**
```
qt.qpa.plugin: Could not find the Qt platform plugin "wayland"
```
**Cause:** The GUI tries to use the Wayland display backend which is not available in this context.  
**Fix:** Force X11 before launching:
```bash
export QT_QPA_PLATFORM=xcb
```

---

### Checkerboard not detected
**Symptoms:** No colored lines appear on the checkerboard in the calibration window.  
**Causes and fixes:**

| Cause | Fix |
|---|---|
| Wrong `--size` parameter | Count **inner corners**, not squares. A 10×7 board → `--size 9x6` |
| Board is warped | Glue it on a rigid flat surface |
| Poor lighting | Use diffuse uniform light, avoid reflections and shadows |
| Board too far or too close | It should fill roughly 1/3 to 1/2 of the image |

---

### `camera_name` mismatch warning
**Symptom:**
```
[WARN] [camera_name] does not match narrow_stereo in file ...yaml
```
**Cause:** The calibration tool always saves `camera_name: narrow_stereo` by default, regardless of the actual camera.  
**Fix:** Edit the YAML file and replace `narrow_stereo` with the correct camera name (see step 4.6).

---

### Camera already in use when launching the full stack
**Symptom:**
```
[ERROR] Component constructor threw an exception: failed to acquire camera
```
**Cause:** A standalone `camera_ros camera_node` process is still running when you launch the apriltag stack. Both processes try to open the same physical camera and the second one fails.  
**Fix:** Stop the standalone camera node before running `ros2 launch openamrobot_docking apriltag.launch.yml`.

---

## 7. What each file does

| File | Role |
|---|---|
| `config/tags_36h11.yaml` | AprilTag parameters: family, size, tag ID, TF frame name |
| `config/docking_pose_publisher.yaml` | Which TF frames to read and which topic to publish the pose on |
| `config/openamrobot_docking.yaml` | Docking server parameters (speed, tolerances, dock position in map) |
| `launch/apriltag.launch.yml` | Camera + rectification + AprilTag detection in one container |
| `launch/openamrobot_docking.launch.py` | Full docking stack (AprilTag + pose publisher + docking server) |
| `src/detected_dock_pose_publisher.cpp` | C++ node: reads `map → charging_dock_apriltag` TF, publishes `PoseStamped` |
| `scripts/dock_trigger.py` | Python node: listens to a Bool topic, sends dock/undock action goals |

---

## 8. Full pipeline diagram

```
Camera (camera_ros)
  └── image_raw
        └── RectifyNode (image_proc)          ← needs calibration
              └── image_rect
                    └── AprilTagNode (apriltag_ros)
                          └── TF: camera → charging_dock_apriltag
                                └── detected_dock_pose_publisher
                                      └── /detected_dock_pose (PoseStamped)
                                            └── opennav_docking server
                                                  └── dock_trigger.py
```
