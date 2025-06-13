import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class IRSensorNode(Node):
    def __init__(self):
        super().__init__('ir_sensor_node')
        self.publisher_ = self.create_publisher(Float32, 'ir_distance', 10)
        self.timer = self.create_timer(0.1, self.publish_distance)

    def publish_distance(self):
        msg = Float32()
        msg.data = self.get_fake_ir_value()  # Remplacer par la vraie lecture capteur
        self.publisher_.publish(msg)
        self.get_logger().info(f'IR Distance: {msg.data} cm')

    def get_fake_ir_value(self):
        return 20.0  # Simulation / remplacer par valeur lue (ex: GPIO)

def main(args=None):
    rclpy.init(args=args)
    node = IRSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

