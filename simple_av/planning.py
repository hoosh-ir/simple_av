import rclpy
from rclpy.node import Node
import json
import os
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Point
import math
from collections import deque
from simple_av_msgs.msg import LocalizationMsg
import numpy as np


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
        self.subscriptionLocation = self.create_subscription(LocalizationMsg, 'simple_av/localization/location', self.location_callback, 10)

        self.pose = PoseStamped()
        self.location = LocalizationMsg()

        self.isPathPlanned = False
        self.path_as_lanes = None # list of lanes from start point to distination
        self.path = None # list of points in order of path_as_lanes
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

    def distance(point1, point2):
        return np.sqrt((point1['x'] - point2['x'])**2 + 
                    (point1['y'] - point2['y'])**2 + 
                    (point1['z'] - point2['z'])**2)

    def vector_from_points(point1, point2):
        return np.array([point2['x'] - point1['x'], 
                        point2['y'] - point1['y'], 
                        point2['z'] - point1['z']])

    def dot_product(vector1, vector2):
        return np.dot(vector1, vector2)
    
    def display_vehicle_position(self, msg_pose, closest_point, closest_lane_name, min_distance):
        self.get_logger().info(
                f'Received Pose :\n'
                f'Position - x: {msg_pose.pose.position.x}, y = {msg_pose.pose.position.y}, z = {msg_pose.pose.position.z}\n'
                f'Closest point: {closest_point.get_point()}\n'
                f'Closest Lane: {closest_lane_name}\n'
                f'Minimum distance - {min_distance}\n'
            )
    
    # Method to get the desiered lane by name, returns the founded Lanelet
    def get_lane_by_name(self, lane_name):
        lane_number = lane_name.replace("lanelet", "")
        lane_number = int(lane_number)
        if lane_number > len(self.map_data):
            return None
        return self.map_data[lane_number - 1]
    
    def input_test(self):
        pose_msg = self.get_pose()
        location = self.get_location()
        closest_point, closest_lane_names, min_distance = location.closest_point, location.closest_lane_names, location.minimal_distance
        self.display_vehicle_position(pose_msg, closest_point, closest_lane_names, min_distance)

    def create_path_of_points(self):
        points = []
        for lane_name in self.path_as_lanes:
            lane_obj = self.get_lane_by_name(lane_name)
            waypoints = lane_obj['waypoints']
            for waypoint in waypoints:
                points.append(waypoint)

        # Remove duplicate points
        points = [points[i] for i in range(len(points)) if i == 0 or (points[i]['x'] != points[i - 1]['x'] or points[i]['y'] != points[i - 1]['y'] or points[i]['z'] != points[i - 1]['z'])]
        self.path = points
    
    def bfs(self, start_lanelet, dest_lanelet):
        if start_lanelet not in self.graph or dest_lanelet not in self.graph:
            return

        queue = deque([(start_lanelet, [start_lanelet])])
        visited = set()
        visited.add(start_lanelet)

        while queue:
            current_lanelet, path = queue.popleft()

            if current_lanelet == dest_lanelet:
                self.path_as_lanes = path
                self.create_path_of_points()

            for next_lanelet in self.graph[current_lanelet]['nextLanes']:
                if next_lanelet not in visited:
                    visited.add(next_lanelet)
                    queue.append((next_lanelet, path + [next_lanelet]))
        
    def get_next_lane(self):
        location = self.get_location()
        current_lane = location.closest_lane_names.data
        self.get_logger().info(f"current lane: {current_lane}")
        self.get_logger().info(f"path: {self.path_as_lanes}")
        if current_lane in self.path_as_lanes:
            index = self.path_as_lanes.index(current_lane)
            if current_lane is self.path_as_lanes[-1]:
                self.get_logger().info(f"final lane")
                return current_lane # current lane is the destination lane
            else:
                return self.path_as_lanes[index + 1] # returns the next lane
        else:
            return -1 # current lane is not in the path

    def get_next_point(self, threshold=2.0):
        location = self.get_location()
        if not location:
            return None
    
        current_closest_point = location.closest_point # geometry msg Point
        dist = location.minimal_distance
        current_lane = self.get_lane_by_name(location.closest_lane_names.data)
        waypoints = current_lane['waypoints']

         # Calculate direction vectors
        direction_to_current_waypoint = self.vector_from_points(previous_waypoint, current_waypoint)
        direction_to_robot = self.vector_from_points(previous_waypoint, robot_position)   
        # algorithm to determine if a certain point was passed, Doing so, we figure out that point was met and we should find a new point
        # By distance? radious?
        
        # reporting the next point
    
    def local_planning(self):
        print(self.path_as_lanes)
        print(len(self.path))
        for p in self.path:
            print(p)
    

    def global_path_planning(self):
        location = self.get_location()
        if location:
            print("debug 0 - path planning ... ")
            start_lanelet = location.closest_lane_names.data
            dest_lanelet = "lanelet103"
            self.bfs(start_lanelet, dest_lanelet) # Creates the path
            if self.path and self.path_as_lanes:
                self.isPathPlanned = True
                print("debug 1 - path of lanes: ", self.path_as_lanes)

    
    def planning(self):
        if not self.isPathPlanned: # path planning has not yet done.
            self.global_path_planning()
        else:  # path planning has been done and the path list is created.
            print("debug 2 - local planning ")
            self.local_planning()
        # publishes: next_point


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