import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
        self.cmd_pub = self.create_publisher(String, '/motor_command', 10)
        self.create_subscription(Int32, '/distance', self.distance_callback, 10)

    def distance_callback(self, msg):
        distance = msg.data
        if distance < 20:
            self.cmd_pub.publish(String(data='MOVE:STOP'))
            self.get_logger().info("Too close: STOP")
        else:
            self.cmd_pub.publish(String(data='MOVE:FWD'))
            self.get_logger().info("Clear path: FORWARD")

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
