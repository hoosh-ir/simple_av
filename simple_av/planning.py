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
from v2x_msgs.msg import CooperativeSignalsMessage
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
        curves = []
        if len(self.points) < 3:
            return curves

        for i in range(1, len(self.points) - 1):
            v1 = self.direction_vector(self.points[i-1], self.points[i])
            v2 = self.direction_vector(self.points[i], self.points[i+1])
            angle = self.angle_between_vectors(v1, v2)
            if angle > self.angle_threshold:
                curve = {}
                curve[angle] = self.points[i]
                curves.append(curve)

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
            'adjacentLanes': lanelet.get('adjacentLanes', []),
        } for lanelet in self.map_data}

        # Create subscriber to /v2x/traffic_signals1  topic
        self.subscriptionPose = self.create_subscription(CooperativeSignalsMessage, '/v2x/traffic_signals1', self.trafficSignal_callback, 10)

        # Create subscriber to /sensing/gnss/pose topic
        self.subscriptionPose = self.create_subscription(PoseStamped, '/sensing/gnss/pose', self.pose_callback, 10)

        # Create subscriber to /localization/location topic
        self.subscriptionLocation = self.create_subscription(LocalizationMsg, 'simple_av/localization/location', self.location_callback, 10)

        # Initialize the publisher
        ## TODO: rename the lookahead_point topic to planned_route
        self.planning_publisher = self.create_publisher(LookAheadMsg, 'simple_av/planning/lookahead_point', 10)

        self.pose = PoseStamped()  # Initialize pose
        self.location = LocalizationMsg()  # Initialize location
        self.trafficSignal = CooperativeSignalsMessage() # Initialize traffic signal

        self.isPathPlanned = False  # Flag to check if the path has been planned
        self.path_as_lanes = None  # List of lanes from start point to destination
        self.path = None  # List of points in order of path_as_lanes
        self.route = None # List of lanes from start point to destination
        self.current_lane_index = 0
        
        self.base_speed = 10.0 # meters/second
        self.speed_limit = 10.0 # meters/second
        self.lookahead_distance = self.base_speed * 2 # meters
        self.stop_distance = self.base_speed * 4 # meters
        self.status = String() # Cruise, Decelerate, PrepareToStop, Turn
        
        self.isCurveFinished = False
        self.isCurveStarted = False
        self.curve_angle = 0.0
        
        self.densify_interval = 2.0 # meters

        self.localization_closest_point_index = 0
        self.initial_lane = None
        self.search_depth = 3

        self.curve_finish_point = None
        
        # self.dest_lanelet = "lanelet149"
        self.dest_lanelet = "lanelet416"
        
    
    def load_map_data(self):
        """
        Load the map data from a JSON file.
        Returns:
            dict: The map data loaded from the JSON file.
        """
        package_share_directory = get_package_share_directory('simple_av')
        json_file_path = os.path.join(package_share_directory, 'resource', 'V3_map.json')
        # Load and read the JSON file
        with open(json_file_path, 'r') as json_file:
            map_data = json.load(json_file)
            return map_data
        
    def trafficSignal_callback(self, msg):
        """
        Callback function to update the pose data.
        Args:
            msg (PoseStamped): The pose message received from the topic.
        """
        self.trafficSignal = msg

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
            waypoints = lane_obj['dense_waypoints']
            for waypoint in waypoints:
                points.append(waypoint)

        # Densify path of points
        # points = self.densify_waypoints(points)

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
                    for next_adj in self.graph[next_lanelet]['adjacentLanes']:
                        if next_adj not in visited:
                            visited.add(next_adj)
                            queue.append((next_adj, path + [next_adj]))
        

    def find_lookahead_point(self, vehicle_pose, current_closest_point_index, search_area): 
        """
        Get the next point for the vehicle to move towards.
        Args:
            vehicle_pose (dict): The current pose of the vehicle.
            current_closest_point_index (int): The index of the current closest point in the path.
            search_area (list): The list of the points in the search area.
        Returns:
            tuple: The updated closest point index and the next point.
        """
        search_area_indexes_on_path = (self.path.index(search_area[0]), self.path.index(search_area[-1]))
        if current_closest_point_index == len(self.path) - 1:
            print("debug shiit: ", len(self.path))
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
            self.get_logger().error("first_ahead_point >= len(self.path)")
            return first_ahead_point, self.path[first_ahead_point]
        
        # print(f'first_ahead_point = {first_ahead_point}, end of search area = {search_area_indexes_on_path[1]}')
        # find the lookahead point in front of the vehicle.  lookahead distance - interval < look ahead point distance <= lookahead distance
        for i in range(first_ahead_point, search_area_indexes_on_path[1]):
            dist = self.calculate_distance(vehicle_pose, self.path[i])
            if dist <= self.lookahead_distance and dist >= self.lookahead_distance - self.densify_interval:
                return i, self.path[i]
        
        self.get_logger().error("--")
        # return first_ahead_point, self.path[first_ahead_point]
        # TODO: modify this part
        if len(self.path) - first_ahead_point - 1 <= 10:
            return len(self.path) - 1, self.path[-1]
        return first_ahead_point + 10, self.path[first_ahead_point + 10]

    def adjust_speed_to_curve(self, curve_angle):
        # return self.base_speed
        if curve_angle >= 0.25:
            return float(math.ceil(self.base_speed / 3.0))
        else:
            return float(math.ceil(self.base_speed / 2))


    def curve_detector(self, curves, look_ahead_point, look_ahead_point_index):
        
        if look_ahead_point_index >= len(self.path) - 5 and look_ahead_point_index <= len(self.path):
            return False, 0.0

        if not self.isCurveStarted and not self.isCurveFinished:
            for curve in curves:
                k, v = next(iter(curve.items()))
                if self.path[look_ahead_point_index - 2] == v or self.path[look_ahead_point_index - 1] == v or self.path[look_ahead_point_index] == v or self.path[look_ahead_point_index+1] == v or self.path[look_ahead_point_index+2] == v:
                    print("debug - curve started", look_ahead_point_index)
                    self.curve_angle = k
                    self.curve_finish_point = self.path[look_ahead_point_index + int(self.lookahead_distance//self.densify_interval)]
                    self.isCurveStarted = True
                    self.isCurveFinished = False
                    # self.isCurveDetected = True
                    return True, self.curve_angle    
        if self.isCurveStarted and not self.isCurveFinished:
            vehicle_pose = {'x': self.pose.pose.position.x, 'y': self.pose.pose.position.y, 'z': self.pose.pose.position.z}
            if self.calculate_distance(vehicle_pose, self.curve_finish_point) <= self.densify_interval * 2:
                print("debug - curve finished")
                self.isCurveFinished = True
                self.isCurveStarted = True
                return False, 0.0
            return True, self.curve_angle
        if self.isCurveStarted and self.isCurveFinished:
            self.isCurveStarted = False
            self.isCurveFinished = False
            # self.isCurveDetected = False
            return False, 0.0
        return False, 0.0


    def curve_handler(self, look_ahead_point, look_ahead_point_index):
        path_curve_detector = PathCurveDetector(self.path, angle_threshold=3) # initializing object from class
        curves = path_curve_detector.find_curves_in_path() # locating curves on the route/path
        
        isTurnDetected, curve_angle = self.curve_detector(curves, look_ahead_point, look_ahead_point_index)
        
        if isTurnDetected:
            self.speed_limit = self.adjust_speed_to_curve(curve_angle)
        else:
            self.speed_limit = self.base_speed
        
        return isTurnDetected
    
    def create_search_area(self):
        
        try:
            lane_index = self.route.index(self.location.closest_lane_names.data)
        except:
            # vehicle is out of path
            self.get_logger().warning("Vehicle is out of the Path")
            lane_index = self.current_lane_index
        if lane_index in range(self.current_lane_index, self.current_lane_index + self.search_depth):
            self.current_lane_index = lane_index
        search_area_as_lanes = self.path_as_lanes[self.current_lane_index: self.current_lane_index + self.search_depth]

        # convert lanes in the search are into a list of waypoints
        search_area = []
        for lane in search_area_as_lanes:
            lane_obj = self.find_lane_by_name(lane)
            waypoints = lane_obj['dense_waypoints']
            for waypoint in waypoints:
                search_area.append(waypoint)
        # print("debug - search area as lanes", search_area_as_lanes, "size of search area: ", len(search_area))
        return search_area, search_area_as_lanes

    def find_closest_waypoint_to_vehicle(self, vehicle_pose, search_area):
        # Finding the index of the closest point in search area

        distances_to_vehicle = []
        for waypoint in search_area:
            distances_to_vehicle.append(self.calculate_distance(waypoint, vehicle_pose))
        closest_waypoint_to_vehicle = search_area[distances_to_vehicle.index(min(distances_to_vehicle))]
        current_closest_point_to_vehicle = self.path.index(closest_waypoint_to_vehicle)
        return current_closest_point_to_vehicle


    def local_planning(self, search_area, search_area_as_lanes):
        """
        Perform local path planning to determine the next point for the vehicle.
        """
        if self.pose.pose.position.x == 0.0 and self.pose.pose.position.y == 0.0 and self.pose.pose.position.z == 0.0:
            self.get_logger().warning("Vehicle Pose is not accessible")
            return None, None
        vehicle_pose = {'x': self.pose.pose.position.x, 'y': self.pose.pose.position.y, 'z': self.pose.pose.position.z}
        
        current_closest_point_to_vehicle = self.find_closest_waypoint_to_vehicle(vehicle_pose, search_area)
        
        look_ahead_point_index, look_ahead_point = self.find_lookahead_point(vehicle_pose, current_closest_point_to_vehicle, search_area)
        
        for i in range(len(self.path[look_ahead_point_index-15:])):
            if self.path[i]['x'] == self.location.closest_point.x and self.path[i]['y'] == self.location.closest_point.y and self.path[i]['z'] == self.location.closest_point.z:
                self.localization_closest_point_index = i
            
        return look_ahead_point_index, look_ahead_point
    
    def get_trafficSignals(self):
        v2i_traffic_signals_id = []
        v2i_traffic_signals_colors = []
        if self.trafficSignal:
            signals = self.trafficSignal.traffic_signals.signals
            for signal in signals:
                map_primitive_id = signal.map_primitive_id
                # Each signal has a list of lights
                for light in signal.lights:
                    color = light.color
                    break
                v2i_traffic_signals_id.append(map_primitive_id)
                v2i_traffic_signals_colors.append(color)
            return v2i_traffic_signals_id, v2i_traffic_signals_colors
    
    def manage_traffic_lights(self, look_ahead_point, look_ahead_point_index, search_area_as_lanes):
        v2i_traffic_signals_id, v2i_traffic_signals_colors = self.get_trafficSignals()

        current_lane = self.route[self.current_lane_index]
        lane_obj = self.find_lane_by_name(current_lane)
        current_lane_traffic_light_id = lane_obj['trafficlightsWayIDs']
        
        isTrafficLightDetected = False
        task = None
        _color = None
        if current_lane_traffic_light_id and current_lane_traffic_light_id[0] in v2i_traffic_signals_id:
            color = v2i_traffic_signals_colors[v2i_traffic_signals_id.index(current_lane_traffic_light_id[0])]
            print(f"traffic light detected, lane = {current_lane}, lightID = {current_lane_traffic_light_id[0]}, color = {color}")
            p1 = lane_obj['stopLinePoseP1']
            p2 = lane_obj['stopLinePoseP2']
            if color == 1:
                # self.get_logger().info("Red")
                isTrafficLightDetected = True
                task = 'Stop_red'
                _color = 'red'
            elif color == 3:
                # self.get_logger().info("Green")
                isTrafficLightDetected = True
                task = 'Cruise_green'
                _color = 'green'
            elif color == 2:
                # self.get_logger().info('Amber')
                isTrafficLightDetected = True
                task = 'Stop_amber'
                _color = 'amber'
            else:
                self.get_logger().error("Unkown traffic light color")
                isTrafficLightDetected = True
                task = 'Unknown'
                _color = 'unkown'
        else:
            # self.get_logger().info("no traffic light")
            isTrafficLightDetected = False
            task = 'Cruise'
            _color = 'unkown'
            p1 = None
            p2 = None
        return isTrafficLightDetected, task, _color, p1, p2


    def behavioural_planning(self, look_ahead_point, look_ahead_point_index, search_area, search_area_as_lanes):
        
        # print("behavioural planning ... ")
        vehicle_pose = {'x': self.pose.pose.position.x, 'y': self.pose.pose.position.y, 'z': self.pose.pose.position.z}
        
        isTurnDetected = self.curve_handler(look_ahead_point, look_ahead_point_index)
        isTrafficLightDetected, vehilceTaskForTrafficLight, trafficLightColor, p1, p2 = self.manage_traffic_lights(look_ahead_point, look_ahead_point_index, search_area_as_lanes)
        
        stop_point = Point(x=self.path[-1]['x'], y=self.path[-1]['y'], z=self.path[-1]['z'])
        if isTrafficLightDetected:
            new_x = (p1[0] + p2[0])/2
            new_y = (p1[1] + p2[1])/2
            new_z = (p1[2] + p2[2])/2
            stop_point = Point(x=new_x, y=new_y, z=new_z)
            print("traffic light detected, distance to stop, ", self.calculate_distance(vehicle_pose, {'x': stop_point.x, 'y': stop_point.y, 'z': stop_point.z}))
        
        distance_to_stop_point = self.calculate_distance(vehicle_pose, {'x': stop_point.x, 'y': stop_point.y, 'z': stop_point.z})

        if distance_to_stop_point <= self.densify_interval * 2:
            if isTrafficLightDetected:
                if trafficLightColor == "green":
                    self.status.data = 'Cruise_green'
                else:
                    self.status.data = 'Park'
            else:
                self.status.data = 'Park'
        elif distance_to_stop_point <= self.stop_distance and look_ahead_point_index > len(self.path) - (self.stop_distance / self.densify_interval + 1):
            self.status.data ='Decelerate'
        elif isTrafficLightDetected and isTurnDetected:
            if trafficLightColor == 'green' or trafficLightColor == 'unkown':
                self.status.data = 'Turn'
            else:
                self.status.data = vehilceTaskForTrafficLight
        elif isTrafficLightDetected:
            if vehilceTaskForTrafficLight == 'Unknown':
                vehilceTaskForTrafficLight = 'Cruise_green'
            self.status.data = vehilceTaskForTrafficLight
        elif isTurnDetected:
            self.status.data = 'Turn'
        else:
            self.status.data = 'Cruise'
        
        return stop_point
           

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
                self.initial_lane = self.location.closest_lane_names.data
                self.route = self.path_as_lanes[:]
                self.current_lane_index = 0
                # print("path of lanes: ", self.path)

  
    def planning(self):
        """
        Main planning function to decide between global and local planning.
        """
        if not self.isPathPlanned:
            print("path planning ... ")
            self.mission_planning()  # generates the path and dencifies it.
        else:
            if not self.location and not self.pose:
                print("error - no location or pose input")
                return None
            
            if self.initial_lane != self.path_as_lanes[0]:
                self.get_logger().error("The start lane from localization Node is not the first lane in path")
                return
            
            search_area, search_area_as_lanes = self.create_search_area()
            
            look_ahead_point_index, look_ahead_point = self.local_planning(search_area, search_area_as_lanes)
            if not look_ahead_point and not look_ahead_point_index:
                return

            stop_point = self.behavioural_planning(look_ahead_point, look_ahead_point_index, search_area, search_area_as_lanes)
            
            # publishing
            lookahead_point = LookAheadMsg()
            lookahead_point.look_ahead_point = Point(x=look_ahead_point['x'], y=look_ahead_point['y'], z=look_ahead_point['z'])
            lookahead_point.stop_point = stop_point
            lookahead_point.status = self.status
            lookahead_point.speed_limit = self.speed_limit
        
            print(self.status.data, self.location.closest_lane_names.data, self.route[self.current_lane_index], look_ahead_point_index, self.localization_closest_point_index, len(self.path), self.speed_limit)
            self.planning_publisher.publish(lookahead_point)
    

def main(args=None):
    rclpy.init(args=args)
    node = Planning()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=None)# Set timeout to 0 to avoid delay
            node.planning()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()