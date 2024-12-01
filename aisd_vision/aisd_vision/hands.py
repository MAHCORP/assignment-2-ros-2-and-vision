import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
mp_hands = mp.solutions.hands

from aisd_msgs.msg import Hand

class Hands(Node):
    def __init__(self):
        super().__init__('hands')
        self.subscription = self.create_subscription(
            Image, 'video_frames', self.listener_callback, 10)
        self.subscription
        self.br = CvBridge()
        self.hand_publisher = self.create_publisher(Hand, 'cmd_hand', 10)

    def listener_callback(self, msg):
        image = self.br.imgmsg_to_cv2(msg)

        PINKY_FINGER_TIP = 20
        INDEX_FINGER_TIP = 8
        # Analyse the image for hands
        with mp_hands.Hands(
            model_complexity=0,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5) as myhands:

            image.flags.writeable = False
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = myhands.process(image)
            if results.multi_hand_landmarks:
                msg = Hand()
                msg.xpinky = results.multi_hand_landmarks[0].landmark[PINKY_FINGER_TIP].x
                msg.xindex = results.multi_hand_landmarks[0].landmark[INDEX_FINGER_TIP].x
                if self.hand_publisher.get_subscription_count() > 0:
                    self.hand_publisher.publish(msg)
                else:
                    self.get_logger().info('waiting for subcriber')

def main(args=None):
    rclpy.init(args=args)
    hands = Hands()
    rclpy.spin(hands)
    hands.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
