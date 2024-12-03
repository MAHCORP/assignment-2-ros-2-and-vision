import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image  # ROS Image message type
from cv_bridge import CvBridge  # OpenCV and ROS image conversion
import cv2  # OpenCV library for image processing
import mediapipe as mp  # Mediapipe library for hand tracking
from aisd_msgs.msg import Hand  # Custom ROS message type for hand data

# Initialize Mediapipe Hands module
mp_hands = mp.solutions.hands

class Hands(Node):
    def __init__(self):
        super().__init__('hands')  # Node name: hands
        
        # A subscriber to the 'image_topic' topic. This will receive image data from ImagePublisher.
        self.subscription = self.create_subscription(
            Image,
            'image_topic',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning

        # A converter for transforming ROS image messages into OpenCV format
        self.br = CvBridge()

        # A publisher for sending Hand messages to the 'cmd_hand' topic
        self.hand_publisher = self.create_publisher(Hand, 'cmd_hand', 10)

        self.get_logger().info('Hands node initialized.')

    def listener_callback(self, msg):
        """
        Callback function that processes images and analyzes hand positions.
        """
        # Convert ROS Image message to an OpenCV image
        image = self.br.imgmsg_to_cv2(msg)

        # Define landmarks for the pinky finger tip and index finger tip
        PINKY_FINGER_TIP = 20
        INDEX_FINGER_TIP = 8

        # Initialize Mediapipe Hands processing
        with mp_hands.Hands(
            model_complexity=0,  # Lightweight model
            min_detection_confidence=0.5,  # Minimum detection confidence threshold
            min_tracking_confidence=0.5  # Minimum tracking confidence threshold
        ) as myhands:

            # Optimize processing by marking the image as non-writeable
            image.flags.writeable = False
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)  # Convert image to RGB format

            # Process the image to detect hands
            results = myhands.process(image)

            # Check if any hand landmarks were detected
            if results.multi_hand_landmarks:
                # Extract the landmarks for the first detected hand
                landmarks = results.multi_hand_landmarks[0].landmark

                # Create a Hand message and populate it with data
                hand_msg = Hand()
                hand_msg.xpinky = landmarks[PINKY_FINGER_TIP].x  # Pinky finger x-coordinate
                hand_msg.xindex = landmarks[INDEX_FINGER_TIP].x  # Index finger x-coordinate

                # Publish the hand data if there are subscribers to the cmd_hand topic
                if self.hand_publisher.get_subscription_count() > 0:
                    self.hand_publisher.publish(hand_msg)
                    self.get_logger().info(f'Published hand data: xpinky={hand_msg.xpinky}, xindex={hand_msg.xindex}')
                else:
                    self.get_logger().info('No subscribers for cmd_hand topic, waiting...')

def main(args=None):
    # Initialize the ROS client library
    rclpy.init(args=args)

    # Create the Hands node
    hands_node = Hands()

    # Keep the node running to process callbacks
    rclpy.spin(hands_node)

    # Destroy the node explicitly when exiting
    hands_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
