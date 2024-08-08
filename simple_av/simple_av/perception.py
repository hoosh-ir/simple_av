import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Point
from simple_av_msgs.msg import LocalizationMsg
from v2x_msgs.msg import CooperativeSignalsMessage


class Perception(Node):
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
        # self.planning_publisher = self.create_publisher(LookAheadMsg, 'simple_av/planning/lookahead_point', 10)

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
    
    def perception(self):
        pass

        




def main(args=None):
    rclpy.init(args=args)
    node = Perception()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=None)# Set timeout to 0 to avoid delay
            node.perception()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()