import rclpy
from rclpy.node import Node
import serial
import threading

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        self.get_logger().info('Serial connecté sur /dev/ttyACM0')

        threading.Thread(target=self.read_serial, daemon=True).start()

    def read_serial(self):
        while rclpy.ok():
            if self.ser.in_waiting:
                line = self.ser.readline().decode('utf-8').strip()
                self.get_logger().info(f'Reçu : {line}')

def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
