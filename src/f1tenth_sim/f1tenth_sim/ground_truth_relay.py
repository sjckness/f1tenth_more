"""Car ground truth as a PoseStamped next to the person's and the obstacle's.

The car's true pose already reaches ROS as nav_msgs/Odometry on
/sim/ground_truth (the URDF's OdometryPublisher, world pose, bridged by
ros_gz_bridge.yaml). The person and obstacle models carry a PosePublisher,
bridged straight to /sim/ground_truth/{person,obstacle} as PoseStamped. This
node republishes the car's pose the same way, so a test reads all three from

  /sim/ground_truth/car       base_link (rear axle) of the robot
  /sim/ground_truth/person    the spawned person, at its feet
  /sim/ground_truth/obstacle  the spawned obstacle, at its base

and can compute the real clearance between the car's outline and the person
without trusting perception. Measuring only, never read by the stack.
"""

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class GroundTruthRelay(Node):

    def __init__(self):
        super().__init__('sim_ground_truth_relay')
        self.pub = self.create_publisher(PoseStamped, '/sim/ground_truth/car', 10)
        self.create_subscription(Odometry, '/sim/ground_truth', self._on_odom, 10)

    def _on_odom(self, msg: Odometry):
        out = PoseStamped()
        out.header = msg.header
        out.pose = msg.pose.pose
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
