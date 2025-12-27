import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import time
import math

class FakeRadar(Node):
    def __init__(self):
        super().__init__('fake_radar_driver')

        # Publishers matching the ESP32 topics
        self.angle_pub = self.create_publisher(Int32, '/radar/angle', 10)
        self.dist_pub = self.create_publisher(Int32, '/radar/distance', 10)

        self.current_angle = 0
        self.direction = 1

        # Timer to run at 50Hz (similar to the loop speed of ESP32)
        self.create_timer(0.02, self.timer_callback)
        print("Fake Radar Started! Simulating an object at 1 meter...")

    def timer_callback(self):
        # 1. Update Angle (Sweep 0 to 180)
        self.current_angle += (self.direction * 2)
        if self.current_angle >= 180:
            self.current_angle = 180
            self.direction = -1
        elif self.current_angle <= 0:
            self.current_angle = 0
            self.direction = 1

        # 2. Calculate Fake Distance
        # Let's simulate a flat wall 1 meter in front of the robot
        # Geometry: Distance = Wall_Dist / sin(angle)
        angle_rad = math.radians(self.current_angle)

        if 30 < self.current_angle < 150:
            # Create a "Wall" effect
            simulated_dist = 500 / math.sin(angle_rad) # 500mm away
        else:
            # Open space / Out of range
            simulated_dist = 2000

        # Add a little random noise (jitter) to make it look real
        # simulated_dist += random.randint(-10, 10)

        # 3. Publish
        msg_angle = Int32()
        msg_angle.data = int(self.current_angle)
        self.angle_pub.publish(msg_angle)

        msg_dist = Int32()
        msg_dist.data = int(simulated_dist)
        self.dist_pub.publish(msg_dist)

def main():
    rclpy.init()
    node = FakeRadar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
