import rclpy
from rclpy.node import Node
import json
import os
from ament_index_python.packages import get_package_share_directory

class Localization(Node):
    def __init__(self):
        super().__init__('localization')
        self.map_data = self.load_map()
        self.map_data = self.map_data["LaneLetsArray"]

    def load_map(self):
        # Get the path to the resource directory
        package_share_directory = get_package_share_directory('inspector')
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

    def localization(self):
        self.display_lane(self.get_lane("TrafficLane423"))


def main(args=None):
    rclpy.init(args=args)
    node = Localization()
    node.localization()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
