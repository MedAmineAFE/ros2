import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import serial
import time

class SerialInterfaceNode(Node):
    def __init__(self):
        super().__init__('serial_interface_node')

        self.serial = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        time.sleep(2)

        # Souscriptions
        self.create_subscription(String, '/motor_command', self.motor_callback, 10)
        self.create_subscription(String, '/distance_request', self.distance_request_callback, 10)
        self.create_subscription(String, '/ir_request', self.ir_request_callback, 10)

        # Publications
        self.pub_distance = self.create_publisher(Int32, '/distance', 10)
        self.pub_ir = self.create_publisher(String, '/ir_sensors', 10)

    def motor_callback(self, msg):
        self.serial.write((msg.data + '\n').encode())

    def distance_request_callback(self, msg):
        self.serial.write(b'GET_DIST\n')
        line = self.serial.readline().decode().strip()
        if line.startswith("DIST:"):
            try:
                dist = int(line.split(":")[1])
                self.pub_distance.publish(Int32(data=dist))
            except:
                self.get_logger().warn("Invalid distance format")

    def ir_request_callback(self, msg):
        self.serial.write(b'GET_IR\n')
        line = self.serial.readline().decode().strip()
        if line.startswith("IR:"):
            self.pub_ir.publish(String(data=line))
        else:
            self.get_logger().warn("Invalid IR response")

def main(args=None):
    rclpy.init(args=args)
    node = SerialInterfaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
