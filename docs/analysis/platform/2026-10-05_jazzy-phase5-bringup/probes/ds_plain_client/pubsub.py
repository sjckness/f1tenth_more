import sys, rclpy
from std_msgs.msg import String
rclpy.init(); n = rclpy.create_node(sys.argv[1])
if sys.argv[1] == 'talker':
    p = n.create_publisher(String, '/chatter', 10)
    n.create_timer(0.5, lambda: p.publish(String(data='x')))
else:
    n.create_subscription(String, '/chatter', lambda m: None, 10)
try:
    rclpy.spin(n)
except BaseException:
    pass
