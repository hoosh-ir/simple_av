import rclpy
from rclpy.node import Node
import json
import os
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Point
import math
from collections import deque
from simple_av_msgs.msg import LocalizationMsg
from simple_av_msgs.msg import LookAheadMsg
import numpy as np


class Planning(Node):
    def __init__(self):
        super().__init__('Planning')
        # Load the Json map
        self.map_data = self.load_map_data()
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

        # Initialize the publisher
        self.planning_publisher = self.create_publisher(LookAheadMsg, 'simple_av/planning/lookahead_point', 10)

        self.pose = PoseStamped()  # Initialize pose
        self.location = LocalizationMsg()  # Initialize location

        self.isPathPlanned = False  # Flag to check if the path has been planned
        self.path_as_lanes = None  # List of lanes from start point to destination
        self.path = None  # List of points in order of path_as_lanes
        
        self.lookahead_distance = 10.0
        self.densify_interval = 2.0
        
    
    def load_map_data(self):
        """
        Load the map data from a JSON file.
        Returns:
            dict: The map data loaded from the JSON file.
        """
        package_share_directory = get_package_share_directory('simple_av')
        json_file_path = os.path.join(package_share_directory, 'resource', 'map.json')
        # Load and read the JSON file
        with open(json_file_path, 'r') as json_file:
            map_data = json.load(json_file)
            return map_data

    def pose_callback(self, msg):
        """
        Callback function to update the pose data.
        Args:
            msg (PoseStamped): The pose message received from the topic.
        """
        self.pose = msg

    def location_callback(self, msg):
        """
        Callback function to update the location data.
        Args:
            msg (LocalizationMsg): The localization message received from the topic.
        """
        self.location = msg

    def current_pose(self):
        """
        Returns:
            PoseStamped: The current pose of the vehicle.
        """
        return self.pose

    def current_location(self):
        """
        Returns:
            LocalizationMsg: The current location of the vehicle.
        """
        return self.location

    def calculate_distance(self, point1, point2):
        """
        Calculate the Euclidean distance between two points.
        Args:
            point1 (dict): The first point with 'x', 'y', 'z' coordinates.
            point2 (dict): The second point with 'x', 'y', 'z' coordinates.
        Returns:
            float: The Euclidean distance between the two points.
        """
        return np.sqrt((point1['x'] - point2['x'])**2 + 
                    (point1['y'] - point2['y'])**2 + 
                    (point1['z'] - point2['z'])**2)

    def calculate_vector(self, point1, point2):
        """
        Calculate the vector from point1 to point2.
        Args:
            point1 (dict): The starting point with 'x', 'y', 'z' coordinates.
            point2 (dict): The ending point with 'x', 'y', 'z' coordinates.
        Returns:
            np.array: The vector from point1 to point2.
        """
        return np.array([point2['x'] - point1['x'], 
                        point2['y'] - point1['y'], 
                        point2['z'] - point1['z']])

    def calculate_dot_product(self, vector1, vector2):
        """
        Calculate the dot product of two vectors.
        Args:
            vector1 (np.array): The first vector.
            vector2 (np.array): The second vector.
        Returns:
            float: The dot product of the two vectors.
        """
        return np.dot(vector1, vector2)
    
    def display_vehicle_position(self, msg_pose, closest_point, closest_lane_name, min_distance):
        self.get_logger().info(
                f'Received Pose :\n'
                f'Position - x: {msg_pose.pose.position.x}, y = {msg_pose.pose.position.y}, z = {msg_pose.pose.position.z}\n'
                f'Closest point: {closest_point.get_point()}\n'
                f'Closest Lane: {closest_lane_name}\n'
                f'Minimum distance - {min_distance}\n'
            )
    
    def find_lane_by_name(self, lane_name):
        """
        Get the lane object by its name.
        Args:
            lane_name (str): The name of the lane.
        Returns:
            dict: The lane object if found, else None.
        """
        lane_number = lane_name.replace("lanelet", "")
        lane_number = int(lane_number)
        if lane_number > len(self.map_data):
            return None
        return self.map_data[lane_number - 1]

    def densify_waypoints(self, waypoints):
        """
        Densifies a list of waypoints by adding interpolated points so that there is 
        approximately one point every `densify_interval` meters.
        
        Args:
            waypoints (list): A list of dictionaries, each with 'x', 'y', and 'z' keys representing
                            a waypoint's coordinates.
            densify_interval (float): The desired distance (in meters) between consecutive waypoints.

        Returns:
            list: A new list of waypoints with additional interpolated points.
        """
        dense_waypoints = []

        for i in range(len(waypoints) - 1):
            start = waypoints[i]
            end = waypoints[i + 1]
            dense_waypoints.append(start)

            distance = self.calculate_distance(start, end)
            num_points = int(distance // self.densify_interval)

            for j in range(1, num_points + 1):
                t = j / num_points
                new_point = {
                    'x': start['x'] + t * (end['x'] - start['x']),
                    'y': start['y'] + t * (end['y'] - start['y']),
                    'z': start['z'] + t * (end['z'] - start['z'])
                }
                dense_waypoints.append(new_point)

        dense_waypoints.append(waypoints[-1])
        return dense_waypoints

    def generate_path_points(self):
        """
        Create a path of points based on the path of lanes.
        """
        points = []
        for lane_name in self.path_as_lanes:
            lane_obj = self.find_lane_by_name(lane_name)
            waypoints = lane_obj['waypoints']
            for waypoint in waypoints:
                points.append(waypoint)

        # Remove duplicate points
        points = [points[i] for i in range(len(points)) if i == 0 or (points[i]['x'] != points[i - 1]['x'] or points[i]['y'] != points[i - 1]['y'] or points[i]['z'] != points[i - 1]['z'])]
        # Densify path of points
        self.path = self.densify_waypoints(points)
    

    def bfs(self, start_lanelet, dest_lanelet):
        """
        Perform Breadth-First Search (BFS) to find a path from start_lanelet to dest_lanelet.
        Args:
            start_lanelet (str): The name of the starting lanelet.
            dest_lanelet (str): The name of the destination lanelet.
        """
        if start_lanelet not in self.graph or dest_lanelet not in self.graph:
            return

        queue = deque([(start_lanelet, [start_lanelet])])
        visited = set()
        visited.add(start_lanelet)

        while queue:
            current_lanelet, path = queue.popleft()

            if current_lanelet == dest_lanelet:
                self.path_as_lanes = path
                self.generate_path_points()

            for next_lanelet in self.graph[current_lanelet]['nextLanes']:
                if next_lanelet not in visited:
                    visited.add(next_lanelet)
                    queue.append((next_lanelet, path + [next_lanelet]))
        

    def find_lookahead_point(self, vehicle_pose, current_closest_point_index): 
        """
        Get the next point for the vehicle to move towards.
        Args:
            vehicle_pose (dict): The current pose of the vehicle.
            current_closest_point_index (int): The index of the current closest point in the path.
            lookahead_distance (float): The distance lookahead_distance to consider the point as reached.
        Returns:
            tuple: The updated closest point index, the next point, and the status.
        """
        if current_closest_point_index == len(self.path) - 1:
            return current_closest_point_index, self.path[current_closest_point_index], "End of waypoints"
            
        # Calculate direction vectors
        direction_vector = self.calculate_vector(self.path[current_closest_point_index], self.path[current_closest_point_index + 1])
        direction_vector_of_robot = self.calculate_vector(self.path[current_closest_point_index], vehicle_pose)   
        first_ahead_point = 0

        # Find the first point ahead of the vehicle
        if self.calculate_dot_product(direction_vector, direction_vector_of_robot) >= 0: # Vehicle is ahead of the point
            first_ahead_point = current_closest_point_index + 1
        else: # Vehicle is Behind of the point
            first_ahead_point = current_closest_point_index
        
        # final point in path
        if first_ahead_point >= len(self.path):
            return first_ahead_point, self.path[first_ahead_point], "End of waypoints"
        
        # find the lookahead point in front of the vehicle. 8 < look ahead point distance <= 10
        for i in range(first_ahead_point, len(self.path)):
            dist = self.calculate_distance(vehicle_pose, self.path[i])
            if dist <= self.lookahead_distance and dist > self.lookahead_distance - self.densify_interval:
                return i, self.path[i], "continue"
            
        return first_ahead_point, self.path[first_ahead_point], "End of waypoints"

    
    def local_planning(self):
        """
        Perform local path planning to determine the next point for the vehicle.
        """
        location = self.current_location()
        vehicle_pose = self.current_pose()
        if not location and not vehicle_pose:
            print("error - no location or pose input")
            return None
        vehicle_pose = {'x': vehicle_pose.pose.position.x, 'y': vehicle_pose.pose.position.y, 'z': vehicle_pose.pose.position.z}
        current_closest_point_to_vehicle = {'x': location.closest_point.x, 'y': location.closest_point.y, 'z': location.closest_point.z}
        # Finding the index of the closest point in path list
        try:
            current_closest_point_index = next(i for i, point in enumerate(self.path) if point == current_closest_point_to_vehicle)
        except StopIteration:
            print("The closest point to the vehicle is not in the list.")
        
        next_point_index, next_point, status = self.find_lookahead_point(vehicle_pose, current_closest_point_index)
        print(next_point_index, next_point, status)
        
        # publishing the next point
        lookahead_point = LookAheadMsg()
        lookahead_point.look_ahead_point = Point(x=next_point['x'], y=next_point['y'], z=next_point['z'])
        self.planning_publisher.publish(lookahead_point)
    

    def global_path_planning(self):
        """
        Perform global path planning to create a path from the current location to the destination.
        """
        location = self.current_location()
        if location:
            print("path planning ... ")
            start_lanelet = location.closest_lane_names.data
            # start_lanelet = "lanelet215"
            dest_lanelet = "lanelet319"
            self.bfs(start_lanelet, dest_lanelet) # Creates the path
            if self.path and self.path_as_lanes:
                self.isPathPlanned = True
                print("path of lanes: ", self.path_as_lanes)

    
    def planning(self):
        """
        Main planning function to decide between global and local planning.
        """
        if not self.isPathPlanned: # path planning has not yet done.
            self.global_path_planning()
        else:  # path planning has been done and the path list is created.
            # print("local planning ...")
            self.local_planning()


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