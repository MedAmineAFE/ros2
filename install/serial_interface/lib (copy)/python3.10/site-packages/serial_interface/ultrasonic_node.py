import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial
import time

class UltrasonicSensorNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_sensor_node')
        self.publisher_ = self.create_publisher(Int32, '/distance', 10)
        self.serial = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        time.sleep(2)  # Allow Arduino time to reset
        self.timer = self.create_timer(0.5, self.read_distance)

    def read_distance(self):
        self.serial.write(b'GET_DIST\n')
        line = self.serial.readline().decode('utf-8').strip()
        if line.startswith("DIST:"):
            try:
                dist = int(line.split(":")[1])
                msg = Int32()
                msg.data = dist
                self.publisher_.publish(msg)
                self.get_logger().info(f"Distance: {dist} cm")
            except ValueError:
                self.get_logger().warn(f"Invalid value received: {line}")

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

