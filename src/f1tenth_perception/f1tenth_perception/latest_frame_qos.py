"""The QoS for camera image and depth inputs that only ever need the newest frame.

KEEP_LAST 1, BEST_EFFORT. A subscriber that does its work in the callback
and is slower than the camera (30 Hz) builds a backlog under the default
KEEP_LAST 10. The executor then takes the OLDEST frame the reader holds, so
every frame waits for the nine queued ahead of it. Measured on the car on
2026-10-14 for yolo_detector_node (stage_timing): the image arrived 52 ms
after capture, and its callback started 373 ms after capture. /camera/detections
went out 428 ms after capture. With depth 1 the reader holds only the newest
frame and older ones are dropped by DDS, not queued. BEST_EFFORT never
back-pressures the publisher, and it matches a RELIABLE publisher such as the
ZED wrapper.

NOT for an input that a message_filters synchronizer has to match against a
slower stream (detection_3d_node's depth and masks wait up to ~0.5 s for
their detections) or for one where every message matters.
"""

from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

LATEST_FRAME_QOS = QoSProfile(
    depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)
