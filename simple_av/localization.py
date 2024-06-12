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

class Localization(Node):
    def __init__(self):
        super().__init__('localization')
        self.map_data = self.load_map()
        self.map_data = self.map_data["LaneLetsArray"]

        self.subscriptionPose = self.create_subscription(
            PoseStamped,
            '/sensing/gnss/pose',
            self.localization,
            10
        )

    def load_map(self):
        # Get the path to the resource directory
        package_share_directory = get_package_share_directory('simple_av')
        json_file_path = os.path.join(package_share_directory, 'resource', 'map.json')
        
        # Load and read the JSON file
        with open(json_file_path, 'r') as json_file:
            map_data = json.load(json_file)
            return map_data

    # Prints the whole json map file
    def display_lanes(self, displayTrafficLight = False):
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
    
    # Method to get the desiered lane by name, returns the founded Lanelet
    def get_lane(self, lane_name):
        lane_number = lane_name.replace("TrafficLane", "")
        lane_number = int(lane_number)
        if lane_number > len(self.map_data):
            return None
        return self.map_data[lane_number - 1]


    def find_closest_point_and_lane(self, current_position):
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

        for lane in self.map_data:
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

    def display(self):
        self.display_lanes()

    def localization(self, msg):
        current_position = Point(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        closest_point, closest_lane_names, min_distance = self.find_closest_point_and_lane(current_position)
        self.get_logger().info(
            f'Received Pose :\n'
            f'Position - x: {msg.pose.position.x}, y = {msg.pose.position.y}, z = {msg.pose.position.z}\n'
            f'Closest point: {closest_point}\n'
            f'Closest Lanes: {closest_point}\n' +
            ''.join(f"{lane}, \n" for lane in closest_lane_names) +
            f'Minimum distance - {min_distance}\n'
        )


def main(args=None):
    rclpy.init(args=args)
    node = Localization()
    # node.display()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
