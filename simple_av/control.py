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
        super().__init__('control')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

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

        # Create publishers for control and gear commands
        self.control_publisher = self.create_publisher(AckermannControlCommand, '/control/command/control_cmd', qos_profile)
        self.gear_publisher = self.create_publisher(GearCommand, '/control/command/gear_cmd', qos_profile)
    

    def control(self):
        pose_message, report_message = self.get_latest_messages()
        if pose_message and report_message:
            self.get_logger().info(
                f'Received Pose :\n'
                f'Position - x: {pose_message.pose.position.x}, y = {pose_message.pose.position.y}, z = {pose_message.pose.position.z}\n'
                f'Received Velocity Report:\n'
                f'sec: {report_message.header.stamp.sec}\n'
                f'Longitudinal Velocity: {report_message.longitudinal_velocity}, Lateral Velocity: {report_message.lateral_velocity}\n'
                f'Heading Rate: {report_message.heading_rate}\n'
            )
            self.get_logger().info("-------------------------------------------------------------------------------------")
    
        # Create and publish control command
        control_msg = AckermannControlCommand()
        control_msg.stamp = self.get_clock().now().to_msg()
        control_msg.lateral = self.get_lateral_command()
        control_msg.longitudinal = self.get_longitudinal_command()
        self.control_publisher.publish(control_msg)

        # Create and publish gear command
        gear_msg = GearCommand()
        gear_msg.stamp = self.get_clock().now().to_msg()
        gear_msg.command = GearCommand.DRIVE
        self.gear_publisher.publish(gear_msg)

    def pose_callback(self, msg):
        self.pose = msg

    def report_callback(self, msg):
        self.report = msg

    def get_latest_messages(self):
        return self.pose, self.report
    
    def get_lateral_command(self):
        lateral_command = AckermannLateralCommand()
        lateral_command.steering_tire_angle = 5.0  # example value
        lateral_command.steering_tire_rotation_rate = 0.5  # example value
        return lateral_command

    def get_longitudinal_command(self):
        longitudinal_command = LongitudinalCommand()
        longitudinal_command.speed = 1.0  # example value
        longitudinal_command.acceleration = 0.05  # example value
        return longitudinal_command
    
    def get_gear_command(self):
        gear = GearCommand()
        gear.stamp=self.get_clock().now().to_msg()
        gear.command=GearCommand.DRIVE



def main(args=None):
    rclpy.init(args=args)
    node = Data_box()

    while rclpy.ok():
        rclpy.spin_once(node)
        node.control()
        
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()