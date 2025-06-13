import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String

class LedControlNode(Node):
    def __init__(self):
        super().__init__('led_control_node')

        # Ouvre le port série (assure-toi que c'est le bon port pour ton Arduino)
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        self.get_logger().info('Serial connecté sur /dev/ttyACM0')

        # Crée un subscriber pour écouter les messages ROS 2
        self.subscriber = self.create_subscription(
            String,  # Type du message
            'led_control',  # Nom du topic
            self.listener_callback,  # Fonction de rappel pour traiter les messages reçus
            10  # Taille de la file d'attente
        )

    def listener_callback(self, msg):
        # Cette fonction est appelée chaque fois qu'un message est reçu
        command = msg.data
        if command == '1':  # Allumer la LED
            self.ser.write(b'1')  # Envoie la commande '1' à l'Arduino
            self.get_logger().info('LED allumée')
        elif command == '0':  # Éteindre la LED
            self.ser.write(b'0')  # Envoie la commande '0' à l'Arduino
            self.get_logger().info('LED éteinte')
        else:
            self.get_logger().warn(f'Commande inconnue reçue: {command}')

def main(args=None):
    rclpy.init(args=args)

    # Crée et exécute le nœud
    node = LedControlNode()

    # Exécute le nœud
    rclpy.spin(node)

    # Nettoyage
    node.destroy_node()
    rclpy.shutdown()

