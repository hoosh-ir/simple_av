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

        self.graph = {lanelet['name']: {
            'waypoints': lanelet['waypoints'],
            'nextLanes': lanelet.get('nextLanes', []),
            'prevLanes': lanelet.get('prevLanes', []),
        } for lanelet in self.map_data}

        # Create subscriber to /sensing/gnss/pose topic
        self.subscriptionPose = self.create_subscription(PoseStamped, '/sensing/gnss/pose', self.pose_callback, 10)

        # Create subscriber to /localization/location topic
        self.subscriptionLocation = self.create_subscription(LocationMsg, 'localization/location', self.location_callback, 10)

        self.pose = PoseStamped()
        self.location = LocationMsg()

        self.isPathPlanned = False
        self.path = None
        self.num_lane_transitions = None
    
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
    
    def subscription_test(self):
        pose_msg = self.get_pose()
        location = self.get_location()
        closest_point, closest_lane_names, min_distance = location.closest_point, location.closest_lane_names, location.minimal_distance
        self.display_vehicle_position(pose_msg, closest_point, closest_lane_names, min_distance)
    
    def bfs(self, start_lanelet, dest_lanelet):
        if start_lanelet not in self.graph or dest_lanelet not in self.graph:
            return None, float('inf')

        queue = deque([(start_lanelet, [start_lanelet])])
        visited = set()
        visited.add(start_lanelet)

        while queue:
            current_lanelet, path = queue.popleft()

            if current_lanelet == dest_lanelet:
                return path, len(path) - 1  # Return path and number of lanelet transitions (length - 1)

            for next_lanelet in self.graph[current_lanelet]['nextLanes']:
                if next_lanelet not in visited:
                    visited.add(next_lanelet)
                    queue.append((next_lanelet, path + [next_lanelet]))

        return None, float('inf')  # If no path found

    def path_planning(self):
        start_lanelet = "lanelet1"
        dest_lanelet = "lanelet252"
        self.path, self.num_transitions = self.bfs(start_lanelet, dest_lanelet)
        self.isPathPlanned = True
    
    def planning(self):
        if self.isPathPlanned:
            pass
        else:
            self.path_planning()
            if self.path and self.num_transitions:
                print(self.path)
                print(self.num_transitions)

def main(args=None):
    rclpy.init(args=args)
    node = Planning()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)  # Set timeout to 0 to avoid delay
            node.planning()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()