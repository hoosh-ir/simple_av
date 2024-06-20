import rclpy
from rclpy.node import Node
import json
import os
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point as GeoPoint
from std_msgs.msg import String
import math
from sensor_msgs.msg import Imu
from simple_av_msgs.msg import LocalizationMsg



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

class Localization(Node):
    def __init__(self):
        super().__init__('localization')
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

        self.subscriptionPose = self.create_subscription(
            PoseStamped,
            '/awsim/ground_truth/vehicle/pose',
            self.groundTruth_callback,
            10
        )

        self.subscriptionPose = self.create_subscription(
            Imu,
            '/sensing/imu/tamagawa/imu_raw',
            self.imu_callback,
            10
        )

        # Initialize the publisher
        self.localization_publisher = self.create_publisher(LocalizationMsg, 'localization/location', 10)

        self.pose_msg = PoseStamped()
        self.ground_truth_msg = PoseStamped()
        self.imu_msg = Imu()
        self.isGlobalPositioningDone = False
        self.local_positioning_depth_search = 2

        # Initialize instance variables for storing closest point and lanes
        self.closest_point = None
        self.closest_lane_name = String()
        self.min_distance = float('inf')

    def load_map(self):
        # Get the path to the resource directory
        package_share_directory = get_package_share_directory('simple_av')
        json_file_path = os.path.join(package_share_directory, 'resource', 'map.json')
        # Load and read the JSON file
        with open(json_file_path, 'r') as json_file:
            map_data = json.load(json_file)
            return map_data

    def groundTruth_callback(self, msg):
        self.ground_truth_msg = msg
    
    def get_groundTruth_msg(self):
        return self.ground_truth_msg

    def pose_callback(self, msg):
        self.pose_msg = msg
    
    def get_pose_msg(self):
        return self.pose_msg
    
    def imu_callback(self, msg):
        self.imu_msg = msg
    
    def get_imu_msg(self):
        return self.imu_msg
    
    def display_map(self, displayTrafficLight = False):
        for lanelet in self.map_data:
            if displayTrafficLight:
                if len(lanelet['trafficlightsWayIDs']) > 0:
                    self.display_lane(lanelet)
            else:
                self.display_lane(lanelet)
  
    # Prints the given Lanelet from the Json map file
    def display_lane(self, lanelet):
        if lanelet is None:
            self.get_logger().info("Lane Not exists")
        else:
            self.get_logger().info(
                f"Lanelet Name: {lanelet['name']}\n"
                "Waypoints:\n" +
                ''.join(f"  x: {waypoint['x']}, y: {waypoint['y']}, z: {waypoint['z']}\n" for waypoint in lanelet['waypoints']) +
                f"Previous Lanes: {lanelet['prevLanes']}\n"
                f"Next Lanes: {lanelet['nextLanes']}\n"
                f"Traffic Lights Way IDs: {lanelet['trafficlightsWayIDs']}\n"
                f"Stop Line Pose P1: {lanelet['stopLinePoseP1']}\n"
                f"Stop Line Pose P2: {lanelet['stopLinePoseP2']}\n"
            )

    def display_vehicle_position(self, msg_pose, closest_point, closest_lane_name, min_distance):
        self.get_logger().info(
                f'Received Pose :\n'
                f'Position - x: {msg_pose.pose.position.x}, y = {msg_pose.pose.position.y}, z = {msg_pose.pose.position.z}\n'
                f'Closest point: {closest_point.get_point()}\n'
                f'Closest Lane: {closest_lane_name}\n'
                f'Minimum distance - {min_distance}\n'
            )
    
    # Method to get the desiered lane by name, returns the founded Lanelet
    def get_lane(self, lane_name):
        lane_number = lane_name.replace("lanelet", "")
        lane_number = int(lane_number)
        if lane_number > len(self.map_data):
            return None
        return self.map_data[lane_number - 1]

    def build_search_area(self, closest_lane_name):
        """
        Builds a search area by exploring connected lanes from the closest lanes identified.

        Args:
        - closest_lane_name (string): name of the closest lane to the vehicle.

        Returns:
        - unique_elements (set): A set of unique lane names forming the search area.

        Explanation:
        - Uses BFS (Breadth-First Search) approach to traverse through connected lanes up to a specified depth.
        - Adds each visited lane to a set to ensure uniqueness in the search area.
        - Returns a set of unique lane names representing the search area surrounding the vehicle.
        """
        search_areas = []
        search_area = set()
        lanes_to_visit = [(closest_lane_name, 0)]
        while lanes_to_visit:
            lane_name, depth = lanes_to_visit.pop(0)
            if lane_name not in search_area and depth <= self.local_positioning_depth_search:
                search_area.add(lane_name)
                for lane in self.map_data:
                    if lane['name'] == lane_name:
                        lanes_to_visit.extend([(next_lane, depth + 1) for next_lane in lane.get('nextLanes', [])])
                        lanes_to_visit.extend([(prev_lane, depth + 1) for prev_lane in lane.get('prevLanes', [])])
        search_areas.append(list(search_area))
        
        unique_elements = set()  # Using a set to store unique elements
        for search_area in search_areas:
            for element in search_area:
                unique_elements.add(element)  # Add each element to the set
        return unique_elements

    def get_closest_point_and_lane(self, current_position, search_lanes=None):
        """
        Finds the closest point(s) to the current_position among the waypoints of specified lanes or all lanes.

        Args:
        - current_position (Point): The point to which distances are compared.
        - search_lanes (list, optional): List of lane names to search for closest points. If None, searches all lanes.

        Returns:
        - closest_points (list): A list of Point objects representing the closest point(s) found.
        - closest_lane_name (string): name of the closest lane to the vehicle
        - min_distance (float): The minimum distance found from current_position to the closest point(s).

        Explanation:
        - Searches for the closest point(s) among specified lanes or all lanes if search_lanes is None.
        - Uses the Point.distance_to method to calculate distances between current_position and waypoints.
        - Updates closest_points, closest_lane_name, and min_distance when a closer point is found.
        - Handles cases where multiple points have the same minimum distance by keeping all of them.
        - Returns the closest point(s), corresponding lane names, and the minimum distance found.
        """
        closest_point = Point()
        closest_lanes_names = []
        closest_lane_name = String()
        min_distance = float('inf')

        lanes_to_search = search_lanes if search_lanes else self.map_data

        for lane in lanes_to_search:
            if search_lanes:
                lane_name = lane
                lanelet_obj = self.get_lane(lane)
                waypoints = lanelet_obj['waypoints']
            else:
                lane_name = lane['name']
                waypoints = lane['waypoints']

            for waypoint in waypoints:
                x = waypoint['x']
                y = waypoint['y']
                z = waypoint['z']
                point = Point(x, y, z)

                distance = current_position.distance_to(point)
                if distance < min_distance:
                    min_distance = distance
                    closest_point = point
                    if closest_lanes_names:
                        closest_lanes_names.clear()
                    closest_lanes_names.append(lane_name)
                elif distance == min_distance:
                    closest_lanes_names.append(lane_name)

        if len(closest_lanes_names) > 1:
            min_avg_distance = float('inf')
            for lane_name in closest_lanes_names:
                acuumulative_distance = 0.0
                lane = self.get_lane(lane_name)
                waypoints = lane['waypoints']
                for waypoint in waypoints:
                    x = waypoint['x']
                    y = waypoint['y']
                    z = waypoint['z']
                    point = Point(x, y, z)
                    acuumulative_distance += current_position.distance_to(point)
                avg_distance = acuumulative_distance / len(waypoints)
                if avg_distance < min_avg_distance:
                    min_avg_distance = avg_distance
                    closest_lane_name = lane_name
        elif len(closest_lanes_names) == 1:
            closest_lane_name = closest_lanes_names[0]

        return closest_point, closest_lane_name, min_distance

    def global_positioning(self):
        """
        Performs global positioning by finding the closest lane and point to the initial vehicle position.

        Returns:
        - closest_point (Point or None): The closest point found in the lane waypoints.
        - closest_lane_name (string): name of the closest lane to the vehicle.
        - min_distance (float): The minimum distance found to the closest point.

        Explanation:
        - Retrieves the current vehicle position from the pose message.
        - Uses get_closest_point_and_lane method to find the closest point(s) among all lanes.
        - Displays the vehicle's position and closest lane information.
        - Returns the closest point(s), corresponding lane names, and minimum distance found.
        """
        pose_msg = self.get_pose_msg()
        if pose_msg and pose_msg.pose.position.x != 0 and pose_msg.pose.position.y != 0 and pose_msg.pose.position.z != 0:
            # print(pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z)
            current_position = Point(pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z)
            closest_point, closest_lane_name, min_distance = self.get_closest_point_and_lane(current_position)
            # self.display_vehicle_position(pose_msg, closest_point, closest_lane_name, min_distance)
            self.isGlobalPositioningDone = True
            return closest_point, closest_lane_name, min_distance
        return None, [], float('inf')
    
    def local_positioning(self, closest_point, closest_lane_name, min_distance):
        """
        Performs local positioning using previous closest lane names to narrow down the search area.

        Args:
        - closest_point (Point): The previously found closest point to the vehicle.
        - closest_lane_name (string): name of the closest lane to the vehicle.
        - min_distance (float): The minimum distance found in the previous positioning.

        Returns:
        - closest_point (Point or None): The closest point found in the local search area.
        - closest_lane_name (string): name of the closest lane to the vehicle.
        - min_distance (float): The minimum distance found to the closest point.

        Explanation:
        - Builds a local search area based on previous closest lane names using build_search_area method.
        - Retrieves the current vehicle position from the pose message.
        - Uses get_closest_point_and_lane method to find the closest point(s) among the local search area.
        - Displays the vehicle's position and closest lane information.
        - Returns the closest point(s), corresponding lane names, and minimum distance found in the local search.
        """
        if not closest_lane_name:
            self.get_logger().info("No closest lane name found, skipping local positioning")
            return closest_point, closest_lane_name, min_distance
        local_search_area = self.build_search_area(closest_lane_name)
        pose_msg = self.get_pose_msg()
        if pose_msg:
            current_position = Point(pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z)
            closest_point, closest_lane_name, min_distance = self.get_closest_point_and_lane(current_position, local_search_area)
            # self.display_vehicle_position(pose_msg, closest_point, closest_lane_name, min_distance)
            return closest_point, closest_lane_name, min_distance
        else:
            return closest_point, closest_lane_name, min_distance

    def get_vehicle_heading():
        pass

    def localization(self):
        """
        Performs localization by first attempting global positioning, then local positioning.

        Explanation:
        - Checks if global positioning has been performed; if not, calls global_positioning to perform it.
        - If already globally positioned, calls local_positioning using previous closest point and lane names.
        - Continues to update self.closest_point, self.closest_lane_name, and self.min_distance accordingly.
        """
        # pose = PoseStamped()
        # pose = self.get_groundTruth_msg()
        # print(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z)
        self.display_imu()
        if not self.isGlobalPositioningDone:
            # self.get_logger().info(f"global positioning, {self.isGlobalPositioningDone}")
            self.closest_point, self.closest_lane_name, self.min_distance = self.global_positioning()
        else:
            # self.get_logger().info("Local positioning")
            self.closest_point, self.closest_lane_name, self.min_distance = self.local_positioning(self.closest_point, self.closest_lane_name, self.min_distance)
            self.publish_vehicle_location(self.closest_point, self.closest_lane_name, self.min_distance)

    def publish_vehicle_location(self, closest_point, closest_lane_name, min_distance):
        localization_result = LocalizationMsg()
        localization_result.closest_point = GeoPoint(x=closest_point.x, y=closest_point.y, z=closest_point.z)
        localization_result.closest_lane_names.data = closest_lane_name
        localization_result.minimal_distance = min_distance
        self.localization_publisher.publish(localization_result)


def main(args=None):
    rclpy.init(args=args)
    node = Localization()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)  # Set timeout to 0 to avoid delay
            node.localization()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
