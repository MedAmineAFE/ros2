import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String

class MotionPlannerNode(Node):
    def __init__(self):
        super().__init__('motion_planner_node')

        self.distance = None
        self.ir_values = [1, 1, 1]  # [left, right, back]
        self.body_seen = False  # ← état mémoire : un corps est-il encore détecté ?

        # Souscriptions
        self.create_subscription(Int32, '/distance', self.distance_callback, 10)
        self.create_subscription(String, '/ir_sensors', self.ir_callback, 10)
        self.create_subscription(String, '/webcam_alert', self.webcam_callback, 10)

        # Publications
        self.motor_pub = self.create_publisher(String, '/motor_command', 10)
        self.distance_req = self.create_publisher(String, '/distance_request', 10)
        self.ir_req = self.create_publisher(String, '/ir_request', 10)

        # Timer principal (boucle toutes les 0.3s)
        self.timer = self.create_timer(0.3, self.main_loop)

    def distance_callback(self, msg):
        self.distance = msg.data

    def ir_callback(self, msg):
        try:
            parts = msg.data.split(":")
            self.ir_values = list(map(int, parts[1].split(",")))
        except Exception as e:
            self.get_logger().error(f"Erreur IR: {e}")

    def webcam_callback(self, msg):
        text = msg.data.strip()
        if text == "BODY_DETECTED":
            self.body_seen = True
        elif text == "CLEAR":
            self.body_seen = False

    def main_loop(self):
        self.distance_req.publish(String())  # Demande /distance
        self.ir_req.publish(String())        # Demande /ir_sensors
        self.make_decision()

    def make_decision(self):
        self.get_logger().info(f"Caméra: {self.body_seen}, Distance: {self.distance}, IR: {self.ir_values}")

        if self.body_seen:
            self.send_command("MOVE:STOP", "👤 Corps encore détecté")
            return

        if self.distance is None:
            return

        left, right, back = self.ir_values
        front_clear = self.distance > 20 and self.distance < 900

        if not front_clear:
            if left == 1:
                self.send_command("MOVE:LEFT", "Obstacle avant, gauche libre")
            elif right == 1:
                self.send_command("MOVE:RIGHT", "Obstacle avant, droite libre")
            elif back == 1:
                self.send_command("MOVE:BWD", "Obstacle avant, recule")
            else:
                self.send_command("MOVE:STOP", "Bloqué de tous les côtés")
        else:
            self.send_command("MOVE:FWD", "Tout est dégagé")

    def send_command(self, cmd, reason):
        self.motor_pub.publish(String(data=cmd))
        self.get_logger().info(f"{reason} → {cmd}")

def main(args=None):
    rclpy.init(args=args)
    node = MotionPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
