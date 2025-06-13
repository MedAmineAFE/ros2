import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String

class MotionPlannerNode(Node):
    def __init__(self):
        super().__init__('motion_planner_node')

        # États internes
        self.distance = None
        self.ir_values = [1, 1, 1]  # [left, right, back]
        self.body_detected = False

        # Abonnements
        self.create_subscription(Int32, '/distance', self.distance_callback, 10)
        self.create_subscription(String, '/ir_sensors', self.ir_callback, 10)
        self.create_subscription(String, '/webcam_alert', self.webcam_callback, 10)

        # Publications
        self.motor_pub = self.create_publisher(String, '/motor_command', 10)
        self.distance_req_pub = self.create_publisher(String, '/distance_request', 10)
        self.ir_req_pub = self.create_publisher(String, '/ir_request', 10)

        # Timer toutes les 0.2s : 1. demande capteurs, 2. prend décision
        self.timer = self.create_timer(0.2, self.main_loop)

    def distance_callback(self, msg):
        self.distance = msg.data

    def ir_callback(self, msg):
        try:
            parts = msg.data.split(":")
            self.ir_values = list(map(int, parts[1].split(",")))
        except Exception as e:
            self.get_logger().error(f"Erreur parsing IR: {e}")

    def webcam_callback(self, msg):
        self.body_detected = (msg.data.strip() == "BODY_DETECTED")

    def main_loop(self):
        # 1. Requête de mise à jour des capteurs
        self.distance_req_pub.publish(String())
        self.ir_req_pub.publish(String())

        # 2. Prise de décision
        self.make_decision()

    def make_decision(self):
        if self.body_detected:
            self.send_command("MOVE:STOP", "Corps détecté")
            return

        if self.distance is None:
            self.get_logger().info("En attente de données de distance...")
            return

        left, right, back = self.ir_values
        front_clear = self.distance > 20

        if not front_clear:
            self.send_command("MOVE:STOP", "Obstacle avant")
        elif left == 0:
            self.send_command("MOVE:RIGHT", "Obstacle gauche")
        elif right == 0:
            self.send_command("MOVE:LEFT", "Obstacle droit")
        elif back == 0:
            self.send_command("MOVE:FWD", "Obstacle arrière")
        else:
            self.send_command("MOVE:FWD", "Route libre")

    def send_command(self, command, reason):
        self.motor_pub.publish(String(data=command))
        self.get_logger().info(f"{reason} — Action: {command}")

def main(args=None):
    rclpy.init(args=args)
    node = MotionPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
