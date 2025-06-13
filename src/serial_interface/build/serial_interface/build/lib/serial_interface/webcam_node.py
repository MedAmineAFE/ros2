import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import time
from flask import Flask, Response
import threading

app = Flask(__name__)
last_frame = None

class WebcamNode(Node):
    def __init__(self):
        super().__init__('webcam_node')
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, 'image_raw', 10)
        self.motor_pub = self.create_publisher(String, '/motor_command', 10)
        self.timer = self.create_timer(1 / 30.0, self.timer_callback)
        self.already_alerted = False

        self.camera = cv2.VideoCapture(0)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(min_detection_confidence=0.5, model_complexity=1)
        self.mp_draw = mp.solutions.drawing_utils

        self.mp_face = mp.solutions.face_detection
        self.face_detection = self.mp_face.FaceDetection(min_detection_confidence=0.5)

    def timer_callback(self):
        global last_frame
        ret, frame = self.camera.read()
        if not ret:
            return

        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Détection de visages
        face_results = self.face_detection.process(img_rgb)
        face_count = 0
        if face_results.detections:
            for detection in face_results.detections:
                bboxC = detection.location_data.relative_bounding_box
                ih, iw, _ = frame.shape
                x = int(bboxC.xmin * iw)
                y = int(bboxC.ymin * ih)
                w = int(bboxC.width * iw)
                h = int(bboxC.height * ih)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, "Face", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                face_count += 1

        if face_count > 0:
            cv2.putText(frame, f"Faces: {face_count}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Détection de la pose
        result = self.pose.process(img_rgb)
        if result.pose_landmarks:
            self.mp_draw.draw_landmarks(frame, result.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)
            if not self.already_alerted:
                msg = String()
                msg.data = "MOVE:STOP"
                self.motor_pub.publish(msg)
                self.get_logger().info("\U0001F6D1 BODY DETECTED — Published MOVE:STOP")
                self.already_alerted = True
        else:
            if self.already_alerted:
                msg = String()
                msg.data = "MOVE:FWD"
                self.motor_pub.publish(msg)
                self.get_logger().info("\u2705 No body — Published MOVE:FWD")
                self.already_alerted = False

        # Mise à jour de l’image pour Flask
        last_frame = frame.copy()

        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.image_pub.publish(image_msg)

    def destroy_node(self):
        self.camera.release()
        super().destroy_node()

def generate_frames():
    global last_frame
    while True:
        if last_frame is None:
            continue
        _, buffer = cv2.imencode('.jpg', last_frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

def main(args=None):
    rclpy.init(args=args)
    node = WebcamNode()

    # Lancer Flask dans un thread
    flask_thread = threading.Thread(target=lambda: app.run(host='0.0.0.0', port=8080, debug=False, use_reloader=False))
    flask_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
