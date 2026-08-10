"""Writes the detected_classes blackboard entry from /camera/detections.

NOT a py_trees.behaviour.Behaviour: there is no per-tick "doing" here, and
tying subscription setup to a BT node's own setup()/update() would mean either
inventing somewhere in the tree to put an always-succeeding node (interfering
with Selector/Sequence arbitration if placed carelessly) or accepting that
freshness only updates on tree ticks instead of on message receipt. Instead this
is a plain object, instantiated once in behavior_executor_node.main() right
after tree.setup() (once tree.node exists) exactly like the bt_loop_duration_ms
parameter is already read post-setup there -- its callback updates the blackboard
directly, independent of tick timing, so ObjectSeen/CheckStopCondition always see
up-to-date data regardless of BT tick rate.

/camera/detections (vision_msgs/Detection2DArray, published by
f1tenth_perception's yolo_detector_node) is the only topic in the workspace that
carries class labels -- confirmed by reading yolo_detector_node.py and
f1tenth_messages/msg/Obstacle2D.msg (the latter is geometry-only: x, y, r, no
class field, despite being the more obvious-looking candidate for this).
"""

import time

import py_trees
from vision_msgs.msg import Detection2DArray

from f1tenth_behavior.mission.runtime import DetectionInfo

DETECTED_CLASSES_KEY = 'detected_classes'


class DetectedClassesBridge:

    def __init__(self, node, detections_topic='/camera/detections'):
        self._node = node
        self.blackboard = py_trees.blackboard.Client(name='DetectedClassesBridge')
        self.blackboard.register_key(
            key=DETECTED_CLASSES_KEY, access=py_trees.common.Access.WRITE)
        setattr(self.blackboard, DETECTED_CLASSES_KEY, {})

        self._sub = node.create_subscription(
            Detection2DArray, detections_topic, self._callback, 10)

    def _callback(self, msg: Detection2DArray):
        now = time.monotonic()
        # Read-modify-write rather than mutating in place: register_key's WRITE
        # access is what py_trees' own diagnostics track as "this client wrote
        # this key this tick/callback", which only fires on an actual setattr.
        classes = dict(getattr(self.blackboard, DETECTED_CLASSES_KEY))
        for det in msg.detections:
            for result in det.results:
                classes[result.hypothesis.class_id] = DetectionInfo(
                    last_seen=now, score=float(result.hypothesis.score))
        setattr(self.blackboard, DETECTED_CLASSES_KEY, classes)
