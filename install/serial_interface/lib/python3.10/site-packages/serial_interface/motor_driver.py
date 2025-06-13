import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')

        # Connexion série vers l'Arduino
        self.serial = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        time.sleep(2)

        # Abonnement à /motor_command
        self.create_subscription(String, '/motor_command', self.motor_callback, 10)

    def motor_callback(self, msg):
        cmd = msg.data.strip()
        self.serial.write((cmd + '\n').encode())
        self.get_logger().info(f"📤 Command sent to Arduino: {cmd}")

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
