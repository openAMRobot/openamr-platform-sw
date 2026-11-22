# SPDX-License-Identifier: MIT
"""station_alignment_stack: station_marker_detector and station_alignment in one process.

The detector hands every station pose to the alignment in process (sources.LocalSink), so the alignment does not
depend on DDS delivery between two processes for its marker input (Baseline 1 run 44247729473: 0 received while the
detector published). /station_alignment/station_pose is still published for other consumers. Both nodes keep their
names and read their sections of the same parameter file.
"""
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter

from openamrobot_station_alignment import sources
from openamrobot_station_alignment.station_alignment_node import StationAlignment
from openamrobot_station_alignment.station_marker_detector import StationMarkerDetector


def build():
    detector = StationMarkerDetector()
    alignment = StationAlignment(
        parameter_overrides=[Parameter("station_pose_source", Parameter.Type.STRING, "local")])
    sink = sources.LocalSink()
    sink.connect(alignment.ingest)
    detector.local_sink = sink
    return detector, alignment


def main(args=None):
    rclpy.init(args=args)
    detector, alignment = build()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(detector)
    executor.add_node(alignment)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        alignment.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
