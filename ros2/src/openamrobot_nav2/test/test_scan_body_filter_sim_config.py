"""
Regression guard for issue #74 / platform-sw#35 (Gate A).

Pure-Python test over scan_body_filter_sim.yaml's *numbers* — no ROS, no Gazebo.
It can't verify Gazebo will actually produce clean returns (only a sim run can),
but it pins down the two things that must never regress:

  1. The mask must cover the re-measured body-reflection band (so the original
     bug — the sim robot seeing its own chassis and refusing to move — can't
     silently come back if someone edits the yaml).
  2. The mask must NOT cover the robot's forward driving direction. After the
     12 Oct 2026 LiDAR mount rotation (rpy 0 0 0 -> 0 0 pi, lidar_joint mounted
     at +x of base_link), the LiDAR's own angle-0 axis flips to point at the
     robot's REAR, not its front: a yaw-pi rotation maps the sensor's local +x
     (scan angle 0) to base_link's -x. So "forward" in the CURRENT scan frame
     is the wraparound pair at the two ends of the array, near +/-180 deg, not
     near 0 deg. This is issue #74's explicit "done when" criterion: a test
     must fail if the mask ever grows to cover the forward sector.

If the LiDAR mount, chassis, or URDF changes again, re-derive FORWARD_CONE_DEG
from the new lidar_joint rpy (see robo_urdf.urdf.xacro) and re-measure
MEASURED_BODY_BAND_DEG from a fresh /scan recording (see scan_body_filter_sim.yaml
for the recording method) rather than editing the numbers below by feel.
"""
import math
import os

import yaml

CONFIG_PATH = os.path.join(
    os.path.dirname(__file__), '..', 'config', 'scan_body_filter_sim.yaml')

# Re-measured 24 Nov 2026 (platform-sw #156) with the simulated LiDAR minimum range
# at 0.05 m (was 0.40 m; 26 Oct 2026 band at 0.40 m: -24.57 to 20.56 deg): raw /scan,
# robot standing in an empty arena, every return under 0.45 m. Body around the mount:
# -38 to +38 deg, 0.124 to 0.273 m. Thin parts: -86 to -84 deg and +84 to +91 deg,
# 0.166 to 0.174 m. The same bands in all 30 launches read (run 45386106021).
MEASURED_BODY_BAND_DEG = (-38.0, 38.0)
MEASURED_THIN_PARTS_DEG = ((-86.0, -84.0), (84.0, 91.0))
MEASURED_THIN_PART_MAX_RANGE_M = 0.174

GAZEBO_XACRO_PATH = os.path.join(
    os.path.dirname(__file__), '..', '..', 'openamrobot_description', 'urdf', 'gazebo_control.xacro')
SIM_LIDAR_MIN_RANGE_M = 0.05   # RPLIDAR S3 datasheet, sensor_profiles.yaml

# lidar_joint rpy is "0 0 pi" (robo_urdf.urdf.xacro, Alex decision 2026-10-12)
# mounted at +x of base_link -> scan angle 0 now points to the robot's REAR,
# and scan angle +/-180 (the array's wraparound ends) points to its FRONT.
# Use a conservative cone either side of the fold so a mask that creeps toward
# it gets caught before it actually reaches the forward-driving direction.
FORWARD_CONE_HALF_WIDTH_DEG = 30.0  # keep [150,180] and [-180,-150] clear


def _load_full_mask_sectors():
    with open(CONFIG_PATH) as f:
        doc = yaml.safe_load(f)
    params = doc['scan_body_filter']['ros__parameters']
    flat = params['full_mask_sectors_deg']
    assert len(flat) % 2 == 0, f'full_mask_sectors_deg must be lo,hi pairs: {flat}'
    return [(flat[i], flat[i + 1]) for i in range(0, len(flat), 2)]


def _covers(sectors, angle_deg):
    return any(lo <= angle_deg <= hi for lo, hi in sectors)


def _overlaps_forward_cone(sectors):
    # Forward is the wraparound pair at the array ends: [180-W, 180] and
    # [-180, -180+W]. A sector overlaps it if any of its boundary or sampled
    # angles fall in either half.
    w = FORWARD_CONE_HALF_WIDTH_DEG
    for lo, hi in sectors:
        # Coarse-sample the sector in 1 deg steps (sectors here are tens of
        # degrees wide at most, so this is cheap and exact enough).
        steps = max(1, int(math.ceil(hi - lo)))
        for i in range(steps + 1):
            a = lo + (hi - lo) * i / steps
            if a >= 180.0 - w or a <= -180.0 + w:
                return True
    return False


def test_mask_covers_the_measured_body_reflection_band():
    sectors = _load_full_mask_sectors()
    lo, hi = MEASURED_BODY_BAND_DEG
    # sample the whole measured band, not just its ends
    steps = 50
    for i in range(steps + 1):
        a = lo + (hi - lo) * i / steps
        assert _covers(sectors, a), (
            f'full_mask_sectors_deg {sectors} does not cover {a:.2f} deg, '
            f'inside the re-measured body-reflection band {MEASURED_BODY_BAND_DEG} '
            '-- issue #74 regression: the sim robot would see its own chassis again')


def test_mask_does_not_cover_the_forward_driving_direction():
    sectors = _load_full_mask_sectors()
    assert not _overlaps_forward_cone(sectors), (
        f'full_mask_sectors_deg {sectors} reaches into the forward driving cone '
        f'(+/-{FORWARD_CONE_HALF_WIDTH_DEG} deg around the wraparound at +/-180 deg) '
        '-- after the 12 Oct 2026 mount rotation that is the direction of travel, '
        "not the robot's own body; masking it at all distances would blind the "
        "robot to real obstacles ahead (issue #74 'done when' criterion)")


def test_close_mask_sectors_also_stay_clear_of_forward():
    with open(CONFIG_PATH) as f:
        doc = yaml.safe_load(f)
    params = doc['scan_body_filter']['ros__parameters']
    flat = params.get('close_mask_sectors_deg') or []
    sectors = [(flat[i], flat[i + 1]) for i in range(0, len(flat), 2)]
    assert not _overlaps_forward_cone(sectors), (
        f'close_mask_sectors_deg {sectors} reaches into the forward driving cone')



def test_close_mask_takes_the_measured_thin_part():
    with open(CONFIG_PATH) as f:
        params = yaml.safe_load(f)['scan_body_filter']['ros__parameters']
    flat = params['close_mask_sectors_deg']
    sectors = [(flat[i], flat[i + 1]) for i in range(0, len(flat), 2)]
    for part in MEASURED_THIN_PARTS_DEG:
        for a in part:
            assert _covers(sectors, a), f'close_mask_sectors_deg {sectors} does not cover {a} deg'
    assert params['close_max'] > MEASURED_THIN_PART_MAX_RANGE_M


def test_the_mask_was_measured_at_the_simulated_lidar_minimum_range():
    # The bands above hold for this minimum range only: a body part nearer than the
    # minimum is invisible, so a lower minimum shows more body. A change of the
    # minimum without a new measurement of the mask must fail here.
    import re
    with open(GAZEBO_XACRO_PATH) as f:
        text = f.read()
    lidar = text[text.index('<lidar>'):text.index('</lidar>')]
    found = [float(v) for v in re.findall(r'<min>([0-9.]+)</min>', lidar)]
    assert found == [SIM_LIDAR_MIN_RANGE_M], found


if __name__ == '__main__':
    test_mask_covers_the_measured_body_reflection_band()
    test_mask_does_not_cover_the_forward_driving_direction()
    test_close_mask_sectors_also_stay_clear_of_forward()
    test_close_mask_takes_the_measured_thin_part()
    test_the_mask_was_measured_at_the_simulated_lidar_minimum_range()
    print('all scan_body_filter_sim.yaml checks passed')
