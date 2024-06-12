#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from autoware_auto_vehicle_msgs.msg import VelocityReport
from autoware_auto_vehicle_msgs.msg import GearCommand
from autoware_auto_control_msgs.msg import AckermannControlCommand, AckermannLateralCommand, LongitudinalCommand
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy  # Import necessary modules


class Data_box(Node):
    def __init__(self):
        super().__init__('gnss_pose_subscriber')

        self.subscriptionPose = self.create_subscription(
            PoseStamped,
            '/sensing/gnss/pose',
            self.pose_callback,
            10
        )
        self.subscriptionReport = self.create_subscription(
            VelocityReport,
            '/vehicle/status/velocity_status',
            self.report_callback,
            10
        )
        self.pose = None
        self.report = None

    def pose_callback(self, msg):
        self.pose = msg

    def report_callback(self, msg):
        self.report = msg

    def get_latest_messages(self):
        return self.pose, self.report



def main(args=None):
    rclpy.init(args=args)
    node = Data_box()

    while rclpy.ok():
        rclpy.spin_once(node)
        pose_msg, report_msg = node.get_latest_messages()
        if pose_msg and report_msg:
            node.get_logger().info(
                f'Received Pose :\n'
                f'Position - x: {pose_msg.pose.position.x}, y = {pose_msg.pose.position.y}, z = {pose_msg.pose.position.z}\n'
                f'Received Velocity Report:\n'
                f'sec: {report_msg.header.stamp.sec}\n'
                f'Longitudinal Velocity: {report_msg.longitudinal_velocity}, Lateral Velocity: {report_msg.lateral_velocity}\n'
                f'Heading Rate: {report_msg.heading_rate}\n'
            )
            node.get_logger().info("-------------------------------------------------------------------------------------")
    
        
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()