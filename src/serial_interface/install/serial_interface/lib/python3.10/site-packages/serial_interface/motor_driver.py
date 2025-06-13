import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial
import time

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
        self.subscriber_ = self.create_subscription(Int32, '/distance', self.listener_callback, 10)
        self.serial = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        time.sleep(2)

    def listener_callback(self, msg):
        distance = msg.data
        if distance < 20:
            self.serial.write(b'MOVE:STOP\n')
            self.get_logger().info("Obstacle detected! Stopping.")
        else:
            self.serial.write(b'MOVE:FWD\n')
            self.get_logger().info("Path is clear. Moving forward.")

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
