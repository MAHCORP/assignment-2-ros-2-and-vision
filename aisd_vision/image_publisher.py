import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.cap = cv2.VideoCapture(0)  # Open the default camera
        self.br = CvBridge()  # Bridge between OpenCV and ROS image format

    def timer_callback(self):
    ret, frame = self.cap.read()  # Capture a frame
    if ret:  # Check if the frame was successfully captured
        self.publisher_.publish(self.br.cv2_to_imgmsg(frame, encoding='bgr8'))  # Publish the image
        self.get_logger().info('Published a video frame')
    else:
        self.get_logger().error('Failed to capture frame')  # Log error if the frame capture fails


    def destroy_node(self):
        """Override to release the video capture when shutting down."""
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    image_publisher = ImagePublisher()

    rclpy.spin(image_publisher)

    # Destroy the node explicitly
    image_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
