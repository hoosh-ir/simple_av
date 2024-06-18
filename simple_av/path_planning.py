import rclpy
from rclpy.node import Node
import json
import os
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
import math
from collections import deque

class Point:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

    def distance_to(self, other_point):
        dx = self.x - other_point.x
        dy = self.y - other_point.y
        dz = self.z - other_point.z
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        return distance

    def get_point(self):
        return self.x, self.y, self.z

class Planning(Node):
    def __init__(self):
        super().__init__('Planning')
        # Load the Json map
        self.map_data = self.load_map()
        self.map_data = self.map_data["LaneLetsArray"]

        # Create subscriber to gnss/pos topic
        self.subscriptionPose = self.create_subscription(
            PoseStamped,
            '/sensing/gnss/pose',
            self.pose_callback,
            10
        )
        

def main(args=None):
    rclpy.init(args=args)
    node = Planning()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()