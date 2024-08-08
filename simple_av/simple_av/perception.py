import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Point
from simple_av_msgs.msg import LocalizationMsg
from v2x_msgs.msg import CooperativeSignalsMessage
from simple_av_msgs.msg import TrafficSignalsArray 


class Perception(Node):
    def __init__(self):
        super().__init__('Perception')

        # Create subscriber to /v2x/traffic_signals1  topic
        self.subscriptionPose = self.create_subscription(CooperativeSignalsMessage, '/v2x/traffic_signals1', self.trafficSignal_callback, 10)

        self.trafficSignal = CooperativeSignalsMessage() # Initialize traffic signal

        # Initialize the publisher
        self.publisher_ = self.create_publisher(TrafficSignalsArray, 'simple_av/perception/traffic_signals', 10)

    def trafficSignal_callback(self, msg):
        """
        Callback function to update the pose data.
        Args:
            msg (PoseStamped): The pose message received from the topic.
        """
        self.trafficSignal = msg

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
        v2i_traffic_signals_id, v2i_traffic_signals_colors = self.get_trafficSignals()

        # Create an instance of the custom message
        msg = TrafficSignalsArray()
        msg.v2i_traffic_signals_id = v2i_traffic_signals_id
        msg.v2i_traffic_signals_colors = v2i_traffic_signals_colors

        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().info('Published traffic signal data')
        

        

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