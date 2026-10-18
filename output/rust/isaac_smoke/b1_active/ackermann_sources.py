import sys, time, rclpy
from ackermann_msgs.msg import AckermannDriveStamped
rclpy.init(); n = rclpy.create_node('ack_sources'); c = {'safety_stop': {}, 'ackermann_drive': {}}
def rec(k):
    def f(m): c[k][m.header.frame_id] = c[k].get(m.header.frame_id, 0) + 1
    return f
n.create_subscription(AckermannDriveStamped, '/safety_stop', rec('safety_stop'), 50)
n.create_subscription(AckermannDriveStamped, '/ackermann_drive', rec('ackermann_drive'), 50)
end = time.monotonic() + float(sys.argv[1])
while time.monotonic() < end: rclpy.spin_once(n, timeout_sec=0.1)
print('SOURCES', c)
