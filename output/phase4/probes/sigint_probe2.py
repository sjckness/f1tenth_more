import os, signal, sys, threading, rclpy
mode = sys.argv[1]
rclpy.init()
n = rclpy.create_node('sigint_probe')
n.create_timer(0.05, lambda: None)
def fire():
    os.kill(os.getpid(), signal.SIGINT)
    if mode == 'double':
        os.kill(os.getpid(), signal.SIGINT)
threading.Timer(1.0, fire).start()
try:
    rclpy.spin(n)
    print(mode, 'spin returned normally')
except KeyboardInterrupt:
    print(mode, 'RAISED KeyboardInterrupt')
except Exception as e:
    print(mode, 'RAISED', type(e).__name__)
