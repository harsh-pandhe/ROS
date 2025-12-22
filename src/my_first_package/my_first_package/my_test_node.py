import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose  # Import the Pose message type

class TurtleMaster(Node):
    def __init__(self):
        super().__init__('turtle_master')
        # Publisher: Sends commands TO the turtle
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # Subscriber: Listens TO the turtle's position
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        
        self.timer = self.create_timer(0.1, self.move_turtle)

    def pose_callback(self, msg):
        # This function runs every time the turtle moves
        self.get_logger().info(f'Turtle is at: X={msg.x:.2f}, Y={msg.y:.2f}')

    def move_turtle(self):
        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = 1.0
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleMaster()
    rclpy.spin(node)
    rclpy.shutdown()