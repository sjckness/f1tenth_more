"""Shaped like mission_logger_node.main(): spin, then cleanup in finally
(on_shutdown + destroy_node + rclpy.shutdown), then write a marker.
No padding: only the destroy calls themselves."""
import sys
import time
import rclpy
from std_msgs.msg import String
from std_srvs.srv import Trigger

def main():
    marker = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node('double_sigint_probe')
    for i in range(5):
        node.create_subscription(String, f'/probe_{i}', lambda m: None, 10)
        node.create_service(Trigger, f'~/srv_{i}', lambda q, r: r)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        open(marker, 'w').write('clean\n')

if __name__ == '__main__':
    main()
