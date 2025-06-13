import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String

class MotionPlannerNode(Node):
    def __init__(self):
        super().__init__('motion_planner_node')

        # État des capteurs
        self.distance = None
        self.ir_values = [1, 1, 1]  # [left, right, back]
        self.body_seen = False  # ← corps humain présent ou non

        # Souscriptions
        self.create_subscription(Int32, '/distance', self.distance_callback, 10)
        self.create_subscription(String, '/ir_sensors', self.ir_callback, 10)
        self.create_subscription(String, '/webcam_alert', self.webcam_callback, 10)

        # Publications
        self.motor_pub = self.create_publisher(String, '/motor_command', 10)
        self.distance_req = self.create_publisher(String, '/distance_request', 10)
        self.ir_req = self.create_publisher(String, '/ir_request', 10)

        # Timer pour lire les capteurs + prendre décision
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
        # Mémorise si un corps est vu ou non
        if msg.data.strip() == "BODY_DETECTED":
            self.body_seen = True
        elif msg.data.strip() == "CLEAR":
            self.body_seen = False

    def main_loop(self):
        # Demande de données à l'Arduino via le serial_interface_node
        self.distance_req.publish(String())
        self.ir_req.publish(String())
        self.make_decision()

    def make_decision(self):
        self.get_logger().info(f"Caméra: {self.body_seen}, Distance: {self.distance}, IR: {self.ir_values}")

        # 1. Priorité absolue à la détection humaine
        if self.body_seen:
            self.send_command("MOVE:STOP", "👤 Corps humain détecté")
            return

        # 2. Attente des données si distance n'est pas encore là
        if self.distance is None:
            return

        # 3. Lecture des capteurs IR
        left, right, back = self.ir_values
        front_clear = self.distance > 20 and self.distance < 900

        # 4. Obstacle à l'avant ?
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

