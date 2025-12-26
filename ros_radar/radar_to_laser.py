import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan
import math
import numpy as np

class RadarToLaser(Node):
    def __init__(self):
        super().__init__('radar_to_laser')
        
        # 1. Listen to ESP32
        self.create_subscription(Int32, '/radar/angle', self.angle_cb, 10)
        self.create_subscription(Int32, '/radar/distance', self.dist_cb, 10)
        
        # 2. Publish Standard LaserScan for RViz
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        
        # 3. Buffer to store the full sweep
        # 0 to 180 degrees = 181 points
        self.current_angle = 0
        self.range_data = [float('inf')] * 181 
        
        # 4. Timer to publish the scan (20Hz)
        self.create_timer(0.05, self.publish_scan)
        
        print("Radar-to-Laser Bridge Started. Ready for RViz!")

    def angle_cb(self, msg):
        # Update current pointer
        if 0 <= msg.data <= 180:
            self.current_angle = msg.data

    def dist_cb(self, msg):
        # Convert mm to meters for ROS standard
        dist_meters = msg.data / 1000.0
        
        # Filter noise/infinity
        if dist_meters > 1.9 or dist_meters < 0.01:
            dist_meters = float('inf')
            
        # Store in buffer
        if 0 <= self.current_angle < 181:
            self.range_data[self.current_angle] = dist_meters

    def publish_scan(self):
        scan = LaserScan()
        
        # Metadata
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'radar_link'  # The name of our sensor in 3D space
        
        # Scan Configuration
        scan.angle_min = 0.0
        scan.angle_max = 3.14159  # 180 degrees in radians
        scan.angle_increment = 3.14159 / 180.0 # 1 degree resolution
        scan.time_increment = 0.0
        scan.scan_time = 0.05
        scan.range_min = 0.02
        scan.range_max = 2.0
        
        # The Data
        scan.ranges = self.range_data
        
        self.scan_pub.publish(scan)

def main():
    rclpy.init()
    node = RadarToLaser()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
