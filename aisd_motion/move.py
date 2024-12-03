import rclpy
from rclpy.node import Node
from aisd_msgs.msg import Hand
from geometry_msgs.msg import Twist

class MoveNode(Node):
    def __init__(self):
        super().__init__('move_node')
        
        # Create a subscriber to the 'cmd_hand' topic that will receive Hand messages
        self.subscription = self.create_subscription(
            Hand,
            'cmd_hand',
            self.listener_callback,
            10  # Queue size
        )
        
        # Create a publisher to the 'cmd_vel' topic that will publish Twist messages
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def listener_callback(self, msg):
        # Default values for linear and angular movement
        angle = 0.0
        linear = 0.0

        # If the xindex of the Hand message is greater than 0.55, move right
        if msg.xindex > 0.55:
            self.get_logger().info('right')
            angle = -0.1  # Rotate clockwise
        # If the xindex of the Hand message is less than 0.45, move left
        elif msg.xindex < 0.45:
            self.get_logger().info('left')
            angle = 0.1  # Rotate counterclockwise
        else:
            angle = 0.0  # No rotation

        # If the xindex is greater than xpinky, move forward
        if msg.xindex > msg.xpinky:
            self.get_logger().info('come')
            linear = 0.5  # Move forward
        else:
            self.get_logger().info('stay')
            linear = 0.0  # Stay still

        # Create a Twist message to control movement
        twist = Twist()
        twist.linear.x = linear  # Set forward/backward speed
        twist.angular.z = angle  # Set rotation speed

        # Publish the Twist message if there's at least one subscriber
        if self.vel_publisher.get_subscription_count() > 0:
            self.vel_publisher.publish(twist)
        else:
            self.get_logger().info('waiting for subscriber')

def main(args=None):
    # Initialize ROS 2
    rclpy.init(args=args)
    
    # Create the MoveNode instance
    move_node = MoveNode()

    # Spin the node to keep it running and processing incoming messages
    rclpy.spin(move_node)

    # Clean up after shutdown
    move_node.destroy_node()
    rclpy.shutdown()
