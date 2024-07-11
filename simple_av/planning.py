import rclpy
from rclpy.node import Node
import json
import os
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
import math
from collections import deque
from simple_av_msgs.msg import LocalizationMsg
from simple_av_msgs.msg import LookAheadMsg
import numpy as np


class PathCurveDetector:
    def __init__(self, points, angle_threshold=3):
        self.points = points
        self.angle_threshold = math.radians(angle_threshold)  # Convert threshold to radians

    @staticmethod
    def direction_vector(p1, p2):
        return (p2['x'] - p1['x'], p2['y'] - p1['y'], p2['z'] - p1['z'])

    @staticmethod
    def vector_magnitude(v):
        return math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)

    @staticmethod
    def dot_product(v1, v2):
        return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]

    @staticmethod
    def angle_between_vectors(v1, v2):
        dot_prod = PathCurveDetector.dot_product(v1, v2)
        mag_v1 = PathCurveDetector.vector_magnitude(v1)
        mag_v2 = PathCurveDetector.vector_magnitude(v2)
        if mag_v1 == 0 or mag_v2 == 0:
            return 0
        cos_theta = dot_prod / (mag_v1 * mag_v2)
        # Ensure the cosine value is within the valid range
        cos_theta = min(1.0, max(-1.0, cos_theta))
        return math.acos(cos_theta)

    def find_curves_in_path(self):
        curves = {}
        if len(self.points) < 3:
            return curves

        for i in range(1, len(self.points) - 1):
            v1 = self.direction_vector(self.points[i-1], self.points[i])
            v2 = self.direction_vector(self.points[i], self.points[i+1])
            angle = self.angle_between_vectors(v1, v2)
            if angle > self.angle_threshold:
                curves[angle] = self.points[i]

        return curves


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
        
        self.base_speed = 10.0 # meters/second
        self.speed_limit = 10.0 # meters/second
        self.lookahead_distance = self.base_speed * 2 # meters
        self.stop_distance = self.base_speed * 4 # meters
        self.status = String() # Cruise, Decelerate, PrepareToStop, Turn
        
        self.isCurveFinished = False
        self.isCurveStarted = False
        self.isCurveDetected = False
        self.densify_interval = 2.0 # meters
        
        self.dest_lanelet = "lanelet192"
        # dest_lanelet = "lanelet319"
        # dest_lanelet = "lanelet335"
        
    
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

    def calculate_distance(self, point1, point2, z=True):
        """
        Calculate the Euclidean distance between two points.
        Args:
            point1 (dict): The first point with 'x', 'y', 'z' coordinates.
            point2 (dict): The second point with 'x', 'y', 'z' coordinates.
        Returns:
            float: The Euclidean distance between the two points.
        """
        if z:
            return np.sqrt((point1['x'] - point2['x'])**2 + 
                        (point1['y'] - point2['y'])**2 + 
                        (point1['z'] - point2['z'])**2)
        else:
            return np.sqrt((point1['x'] - point2['x'])**2 + (point1['y'] - point2['y'])**2)

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

        # Densify path of points
        points = self.densify_waypoints(points)

        # Remove duplicate points
        self.path = [points[i] for i in range(len(points)) if i == 0 or (points[i]['x'] != points[i - 1]['x'] or points[i]['y'] != points[i - 1]['y'] or points[i]['z'] != points[i - 1]['z'])]
    

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
            print("debug 1 - shiit: ", len(self.path))
            return current_closest_point_index, self.path[current_closest_point_index]
            
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
            return first_ahead_point, self.path[first_ahead_point]
        
        # find the lookahead point in front of the vehicle.  lookahead distance - interval < look ahead point distance <= lookahead distance
        for i in range(first_ahead_point, len(self.path)):
            dist = self.calculate_distance(vehicle_pose, self.path[i])
            if dist <= self.lookahead_distance and dist > self.lookahead_distance - self.densify_interval:
                return i, self.path[i]
            
        return first_ahead_point, self.path[first_ahead_point]

    def find_closest_waypoint_to_vehicle(self, vehicle_pose):
        # Finding the index of the closest point in path list
        distances_to_vehicle = []
        for p in self.path:
            distances_to_vehicle.append(self.calculate_distance(p, vehicle_pose))
        current_closest_point_to_vehicle = distances_to_vehicle.index(min(distances_to_vehicle))
        return current_closest_point_to_vehicle

    def update_lookahead_distance(self, curves):
        pass


    def adjust_speed_to_curve(self, curves, curve_angle):
        min_angle = min(curves.keys())
        max_angle = max(curves.keys())
        min_speed = float(math.floor(self.base_speed / 3.0))
        speed = self.base_speed
        
        if curve_angle < min_angle:
            speed = self.base_speed
        elif curve_angle >= max_angle:
            speed = min_speed
        else:
            speed = self.base_speed - 5
        
        return speed


    def curve_detector(self, curves, look_ahead_point, look_ahead_point_index):
        curve_finish_point = {}
        curve_angle = 0.0
        for k, v in curves.items():
                if look_ahead_point == v:
                    curve_angle = k
                    self.isCurveDetected = True
                    self.isCurveStarted = False
                    self.isCurveFinished = False
        if self.isCurveDetected == True:
            if self.isCurveStarted == False and self.isCurveFinished == False:
                self.isCurveStarted = True
                curve_finish_point = self.path[look_ahead_point_index + int(self.lookahead_distance//self.densify_interval)]
                return "Turn", curve_angle
            elif self.isCurveStarted == True and self.isCurveFinished == False:
                vehicle_pose = {'x': self.pose.pose.position.x, 'y': self.pose.pose.position.y, 'z': self.pose.pose.position.z}
                if self.calculate_distance(vehicle_pose, curve_finish_point) <= self.densify_interval:
                    self.isCurveFinished = True
                    return "Cruise", 0.0
                return "Turn", curve_angle
            else:
                self.isCurveStarted = False
                self.isCurveFinished = False
                self.isCurveDetected = False
                return "Cruise", 0.0
        return "Cruise", 0.0


    def adjust_speed(self, curves, look_ahead_point, look_ahead_point_index):
        _status, curve_angle = self.curve_detector(curves, look_ahead_point, look_ahead_point_index)
        if _status == 'Turn':
            self.speed_limit = self.adjust_speed_to_curve(curves, curve_angle)
        else:
            self.speed_limit = self.base_speed
        
        return _status


    def local_planning(self):
        """
        Perform local path planning to determine the next point for the vehicle.
        """

        vehicle_pose = {'x': self.pose.pose.position.x, 'y': self.pose.pose.position.y, 'z': self.pose.pose.position.z}
        current_closest_point_to_vehicle = self.find_closest_waypoint_to_vehicle(vehicle_pose)
        look_ahead_point_index, look_ahead_point = self.find_lookahead_point(vehicle_pose, current_closest_point_to_vehicle)

        return look_ahead_point_index, look_ahead_point
    

    def behavioural_planning(self, look_ahead_point, look_ahead_point_index):
        
        vehicle_pose = {'x': self.pose.pose.position.x, 'y': self.pose.pose.position.y, 'z': self.pose.pose.position.z}
        dist_to_final_waypoint = self.calculate_distance(vehicle_pose, self.path[-1], False)

        path_curve_detector = PathCurveDetector(self.path, angle_threshold=3)
        curves = path_curve_detector.find_curves_in_path()
        _status = self.adjust_speed(curves, look_ahead_point, look_ahead_point_index)

        if dist_to_final_waypoint <= self.stop_distance:
            self.status.data ='Decelerate'
        elif dist_to_final_waypoint <= 2.0:
            self.status.data = 'PrepareToStop'
        else:
            if _status == "Turn":
                self.status.data = 'Turn'
            else:
                self.status.data = 'Cruise'
           

    def mission_planning(self):
        """
        Perform global path planning to create a path from the current location to the destination.
        """
        if self.location:
            print("path planning ... ")
            start_lanelet = self.location.closest_lane_names.data
            self.bfs(start_lanelet, self.dest_lanelet) # Creates the path
            if self.path and self.path_as_lanes:
                self.isPathPlanned = True
                print("path of lanes: ", self.path_as_lanes)
                print("path of lanes: ", self.path)

  
    def planning(self):
        """
        Main planning function to decide between global and local planning.
        """
        if not self.isPathPlanned:
            self.mission_planning()  # generates the path and dencifies it.
        else:
            if not self.location and not self.pose:
                print("error - no location or pose input")
                return None
            
            look_ahead_point_index, look_ahead_point = self.local_planning()

            self.behavioural_planning(look_ahead_point, look_ahead_point_index)
    
            # publishing
            lookahead_point = LookAheadMsg()
            lookahead_point.look_ahead_point = Point(x=look_ahead_point['x'], y=look_ahead_point['y'], z=look_ahead_point['z'])
            lookahead_point.stop_point = Point(x=self.path[-1]['x'], y=self.path[-1]['y'], z=self.path[-1]['z'])
            lookahead_point.status = self.status
            lookahead_point.speed_limit = self.speed_limit

            print("output: ", self.status.data, look_ahead_point_index, self.speed_limit)
            self.planning_publisher.publish(lookahead_point)


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