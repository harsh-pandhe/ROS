import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import numpy as np
import matplotlib.pyplot as plt

class RadarViewer(Node):
    def __init__(self):
        super().__init__('radar_viewer')
        
        # Subscribe to the topics
        self.create_subscription(Int32, '/radar/angle', self.angle_callback, 10)
        self.create_subscription(Int32, '/radar/distance', self.dist_callback, 10)
        
        # Store data: 181 points (0 to 180 degrees)
        self.angles = np.deg2rad(np.arange(0, 181))
        self.distances = np.full(181, 0) # Start with 0 distance
        self.current_angle = 0
        
        print("Radar Viewer Started! Waiting for data...")

    def angle_callback(self, msg):
        # Save the angle the ESP32 is currently pointing at
        self.current_angle = msg.data

    def dist_callback(self, msg):
        # Save the distance at the current angle
        if 0 <= self.current_angle <= 180:
            # Filter out "2000" (infinity/fallback) to make the graph cleaner
            dist = msg.data
            if dist >= 1900: 
                dist = 0 
            self.distances[self.current_angle] = dist

def main():
    rclpy.init()
    node = RadarViewer()

    # Setup the Polar Plot
    plt.ion() # Interactive mode on
    fig = plt.figure(figsize=(6,6))
    ax = fig.add_subplot(111, projection='polar')
    
    # Initial empty line
    line, = ax.plot(node.angles, node.distances, color='green', linewidth=2)
    
    # Style the radar
    ax.set_ylim(0, 1000) # Range: 0 to 1000mm (1 meter)
    ax.set_theta_zero_location("N") # 0 degrees at top
    ax.set_theta_direction(-1)      # Clockwise
    plt.title("ESP32 ROS 2 Radar")

    # Update loop
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        
        # Update the graph
        line.set_ydata(node.distances)
        
        # Draw the "Sweeper" line (red line at current angle)
        # (Optional, but looks cool - requires more code, keeping it simple for now)
        
        fig.canvas.draw()
        fig.canvas.flush_events()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
