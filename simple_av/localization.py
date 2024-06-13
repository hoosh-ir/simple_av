import rclpy
from rclpy.node import Node
import json
import os
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
import math

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
        self.pose_msg = None
        self.isGlobalPositioningDone = False
        self.local_positioning_depth_search = 2

        # Initialize instance variables for storing closest point and lanes
        self.closest_point = None
        self.closest_lane_names = []
        self.min_distance = float('inf')

    def load_map(self):
        # Get the path to the resource directory
        package_share_directory = get_package_share_directory('simple_av')
        json_file_path = os.path.join(package_share_directory, 'resource', 'map.json')
        # Load and read the JSON file
        with open(json_file_path, 'r') as json_file:
            map_data = json.load(json_file)
            return map_data

    def pose_callback(self, msg):
        self.pose_msg = msg
    
    def get_pose_msg(self):
        return self.pose_msg
    
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

    def display_vehicle_position(self, msg_pose, closest_point, closest_lane_names, min_distance):
        self.get_logger().info(
                f'Received Pose :\n'
                f'Position - x: {msg_pose.pose.position.x}, y = {msg_pose.pose.position.y}, z = {msg_pose.pose.position.z}\n'
                f'Closest point: {closest_point.get_point()}\n'
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

    def build_search_area(self, closest_lane_names):
        search_areas = []
        for current_lane_name in closest_lane_names:
            search_area = set()
            lanes_to_visit = [(current_lane_name, 0)]
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

    def find_closest_point_and_lane(self, current_position, search_lanes=None):
        """
        Finds the closest point(s) to the initial_point among the waypoints of all lanes provided in lanes_data.
        
        Args:
        - current_position (Point): The point to which distances are compared.
        
        Returns:
        - closest_points (list): A list of Point objects representing the closest point(s) found.
        - closest_lane_names (list): A list of strings representing the lane name(s) corresponding to the closest point(s).
        - min_distance (float): The minimum distance found from initial_point to the closest point(s).
        
        Comments:
        - Iterates through each lane in lanes_data.
        - For each waypoint in a lane, calculates the distance to initial_point using Point.distance_to method.
        - Updates closest_points, closest_lane_names, and min_distance when a closer point is found.
        - Handles cases where multiple points have the same minimum distance by keeping all of them.
        - If more than one closest point is found, removes points and lane names that do not match the minimum distance.
        """
        closest_points = []
        closest_lane_names = []
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
                    if closest_lane_names and closest_points:
                        closest_lane_names.pop()
                        closest_points.pop()
                    closest_points.append(point)
                    closest_lane_names.append(lane_name)
                elif distance == min_distance:
                    min_distance = distance
                    closest_points.append(point)
                    closest_lane_names.append(lane_name)

        if len(closest_points) > 1:
            points_to_remove = []
            lane_names_to_remove = []
            for i in range(len(closest_points)):
                distance = current_position.distance_to(closest_points[i])
                if distance != min_distance:
                    points_to_remove.append(closest_points[i])
                    lane_names_to_remove.append(closest_lane_names[i])

            for point in points_to_remove:
                closest_points.remove(point)
            for lane in lane_names_to_remove:
                closest_lane_names.remove(lane)

        return closest_points[0], closest_lane_names, min_distance

    # This method calls at the first iteration of this node to find the location of the vehicle. This method compares the position of the starting
    # Point of the vehicle to all the points in lanes in Json map file.
    def global_positioning(self):
        pose_msg = self.get_pose_msg()
        if pose_msg:
            self.isGlobalPositioningDone = True
            current_position = Point(pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z)
            closest_point, closest_lane_names, min_distance = self.find_closest_point_and_lane(current_position)
            self.display_vehicle_position(pose_msg, closest_point, closest_lane_names, min_distance)
            return closest_point, closest_lane_names, min_distance
        return None, [], float('inf')
    
    # Finds the lane, and the point of the vehicle. uses previous positioning values to narrow the search area
    def local_positioning(self, closest_point, closest_lane_names, min_distance):
        if not closest_lane_names:
            self.get_logger().info("No closest lane names found, skipping local positioning")
            return closest_point, closest_lane_names, min_distance
        local_search_area = self.build_search_area(closest_lane_names)
        print(local_search_area)
        pose_msg = self.get_pose_msg()
        if pose_msg:
            current_position = Point(pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z)
            closest_point, closest_lane_names, min_distance = self.find_closest_point_and_lane(current_position, local_search_area)
            self.display_vehicle_position(pose_msg, closest_point, closest_lane_names, min_distance)
            return closest_point, closest_lane_names, min_distance

    def localization(self):
        if not self.isGlobalPositioningDone:
            self.closest_point, self.closest_lane_names, self.min_distance = self.global_positioning()
        else:
            self.closest_point, self.closest_lane_names, self.min_distance = self.local_positioning(self.closest_point, self.closest_lane_names, self.min_distance)

def main(args=None):
    rclpy.init(args=args)
    node = Localization()
    try:
        while rclpy.ok():
            node.isGlobalPositioningDone
            rclpy.spin_once(node, timeout_sec=2.0)  # Set timeout to 0 to avoid delay
            node.localization()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
