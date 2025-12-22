import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class SquareTurtle(Node):
    def __init__(self):
        super().__init__('square_turtle')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.timer = self.create_timer(0.1, self.move_turtle)
        
        self.count = 0
        self.state = "MOVE" # States: MOVE or TURN

    def pose_callback(self, msg):
        # We still listen to the pose, but now we use it for logic
        pass

    def move_turtle(self):
        msg = Twist()
        
        if self.state == "MOVE":
            msg.linear.x = 2.0  # Go forward
            msg.angular.z = 0.0
            self.count += 1
            if self.count > 20: # After 2 seconds (20 * 0.1s)
                self.state = "TURN"
                self.count = 0
        
        elif self.state == "TURN":
            msg.linear.x = 0.0
            msg.angular.z = 1.57 # Roughly 90 degrees in radians
            self.count += 1
            if self.count > 10: # After 1 second
                self.state = "MOVE"
                self.count = 0
                
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SquareTurtle()
    rclpy.spin(node)
    rclpy.shutdown()