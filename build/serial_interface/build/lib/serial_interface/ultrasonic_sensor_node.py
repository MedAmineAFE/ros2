import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32

class UltrasonicSensorNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_sensor_node')
        self.pub = self.create_publisher(String, '/distance_request', 10)
        self.create_subscription(Int32, '/distance', self.distance_callback, 10)
        self.create_timer(0.5, self.send_request)

    def send_request(self):
        self.pub.publish(String(data='GET_DIST'))

    def distance_callback(self, msg):
        self.get_logger().info(f"Distance: {msg.data} cm")

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
