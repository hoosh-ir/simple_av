import rclpy
from rclpy.node import Node
import json
import os
from ament_index_python.packages import get_package_share_directory

class Localization(Node):
    def __init__(self):
        super().__init__('localization')
        self.map_data = self.load_map()

    def load_map(self):
        # Get the path to the resource directory
        package_share_directory = get_package_share_directory('inspector')
        json_file_path = os.path.join(package_share_directory, 'resource', 'map.json')
        
        # Load and read the JSON file
        with open(json_file_path, 'r') as json_file:
            map_data = json.load(json_file)
            return map_data
    
    def display_map(self):
        self.get_logger().info(f'Map data loaded: {self.map_data}')

def main(args=None):
    rclpy.init(args=args)
    node = Localization()
    node.display_map()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
