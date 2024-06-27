#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from autoware_auto_vehicle_msgs.msg import VelocityReport
from autoware_auto_vehicle_msgs.msg import GearCommand
from autoware_auto_control_msgs.msg import AckermannControlCommand, AckermannLateralCommand, LongitudinalCommand
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
import time

class PIDController:
    def __init__(self, p_gain, i_gain, d_gain, target_vel, delta_t=0.01):
        self.kp = p_gain
        self.ki = i_gain
        self.kd = d_gain
        self.target_vel = target_vel
        self.delta_t = delta_t
        self.current_time = time.time()
        self.last_time = self.current_time
        self.integrated_error = 0.0
        self.privous_error = 0.0 # previous tracking error

    def updatePID(self, observed_vel):
        error = self.target_vel - observed_vel # tracking error to the traget velocity
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time

        self.integrated_error += error * delta_time  # integral of the tracking error
        derivative = (error - self.privous_error) / delta_time # derivative of the tracking error

        # calculate control input
        P = self.kp * error
        I = self.ki * self.integrated_error * 0
        D = self.kd * derivative

        acc_cmd = P + I + D

        self.last_time = self.current_time
        self.privous_error = error

        return acc_cmd


class VehicleControl(Node):
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
        
        # PID controller setup
        self.target_speed = 5.0  # target speed in m/s
        self.pid_controller = PIDController(p_gain=0.5, i_gain=0.2, d_gain=0.2, target_vel=self.target_speed)

    def control(self):
        pose_message, report_message = self.get_latest_messages()
            
        # Create and publish control command
        control_msg = AckermannControlCommand()
        control_msg.stamp = self.get_clock().now().to_msg()
        control_msg.lateral = self.get_lateral_command()
        control_msg.longitudinal = self.get_longitudinal_command(report_message.longitudinal_velocity if report_message else 0.0)
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
        lateral_command.steering_tire_angle = 0.0  # example value
        lateral_command.steering_tire_rotation_rate = 0.0  # example value
        return lateral_command

    def get_longitudinal_command(self, current_speed):
        longitudinal_command = LongitudinalCommand()
        longitudinal_command.speed = self.target_speed
        accel = self.pid_controller.updatePID(current_speed)
        longitudinal_command.acceleration = accel
        self.get_logger().info(
                f'speed: {current_speed} :\n'
                f'accel : {accel}\n'
            )
        return longitudinal_command
    
    def get_gear_command(self):
        gear = GearCommand()
        gear.stamp=self.get_clock().now().to_msg()
        gear.command=GearCommand.DRIVE

def main(args=None):
    rclpy.init(args=args)
    node = VehicleControl()

    while rclpy.ok():
        rclpy.spin_once(node)
        node.control()
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()