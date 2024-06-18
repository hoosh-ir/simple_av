import rclpy
from rclpy.node import Node
import json
import os
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Point
import math
from collections import deque
from custom_interface.msg import LocationMsg

class Planning(Node):
    def __init__(self):
        super().__init__('Planning')
        # Load the Json map
        self.map_data = self.load_map()
        self.map_data = self.map_data["LaneLetsArray"]

        # Create subscriber to /sensing/gnss/pose topic
        self.subscriptionPose = self.create_subscription(PoseStamped, '/sensing/gnss/pose', self.pose_callback, 10)

        # Create subscriber to /localization/location topic
        self.subscriptionLocation = self.create_subscription(LocationMsg, 'localization/location', self.location_callback, 10)

        self.pose = PoseStamped()
        self.location = LocationMsg()
    
    def load_map(self):
        # Get the path to the resource directory
        package_share_directory = get_package_share_directory('simple_av')
        json_file_path = os.path.join(package_share_directory, 'resource', 'map.json')
        # Load and read the JSON file
        with open(json_file_path, 'r') as json_file:
            map_data = json.load(json_file)
            return map_data

    def pose_callback(self, msg):
        self.pose = msg

    def location_callback(self, msg):
        self.location = msg

    def get_pose(self):
        return self.pose

    def get_location(self):
        return self.location
    
    def display_vehicle_position(self, msg_pose, closest_point, closest_lane_names, min_distance):
        closest_point = Point(x=closest_point.x, y=closest_point.y, z=closest_point.z)
        self.get_logger().info(
                f'Received Pose :\n'
                f'Position - x: {msg_pose.pose.position.x}, y = {msg_pose.pose.position.y}, z = {msg_pose.pose.position.z}\n'
                f'Closest point: {closest_point}\n'
                f'Closest Lanes:\n' +
                ''.join(f"{lane}, \n" for lane in closest_lane_names) +
                f'Minimum distance - {min_distance}\n'
            )
    
    # Method to get the desiered lane by name, returns the founded Lanelet
    def get_lane(self, lane_name):
        lane_number = lane_name.replace("lanelet", "")
        lane_number = int(lane_number)
        if lane_number > len(self.map_data):
            return None
        return self.map_data[lane_number - 1]

    def path_planning():
        pass

    def test(self):
        pose_msg = self.get_pose()
        location = self.get_location()
        closest_point, closest_lane_names, min_distance = location.closest_point, location.closest_lane_names, location.minimal_distance
        self.display_vehicle_position(pose_msg, closest_point, closest_lane_names, min_distance)

        

def main(args=None):
    rclpy.init(args=args)
    node = Planning()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)  # Set timeout to 0 to avoid delay
            node.test()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()