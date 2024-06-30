#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from autoware_auto_vehicle_msgs.msg import VelocityReport
from autoware_auto_vehicle_msgs.msg import GearCommand
from autoware_auto_control_msgs.msg import AckermannControlCommand, AckermannLateralCommand, LongitudinalCommand
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from simple_av_msgs.msg import LookAheadMsg
import time
import math
from collections import deque

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
        self.slidingWindow = deque(maxlen=10) # for storing only the 10 most recent errors
        self.previous_error = 0.0

    def updatePID(self, observed_vel):
        error = self.target_vel - observed_vel
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        self.slidingWindow.append(error)

        self.integrated_error = sum(self.slidingWindow) * delta_time
        derivative = (error - self.previous_error) / delta_time

        print("debug 0 - sum: ", sum(self.slidingWindow))
        print("debug 0 - delta time: ", delta_time)
        

        P = self.kp * error
        I = self.ki * self.integrated_error
        D = self.kd * derivative
        
        print("P: ", P, " I: ", I, " D: ", D)

        acc_cmd = P + I + D

        self.last_time = self.current_time
        self.previous_error = error

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
        self.subscriptionPose = self.create_subscription(
            PoseStamped,
            '/awsim/ground_truth/vehicle/pose',
            self.ground_truth_callback,
            10
        )
        self.subscriptionVelocityReport = self.create_subscription(
            VelocityReport,
            '/vehicle/status/velocity_status',
            self.velocity_report_callback,
            10
        )
        self.subscriptionLookahead = self.create_subscription(
            LookAheadMsg,
            '/simple_av/planning/lookahead_point',
            self.lookahead_callback,
            10
        )

        self.pose = PoseStamped()
        self.ground_truth = PoseStamped()
        self.velocity_report = VelocityReport()
        self.lookahead_point = LookAheadMsg()

        self.control_publisher = self.create_publisher(AckermannControlCommand, '/control/command/control_cmd', qos_profile)
        self.gear_publisher = self.create_publisher(GearCommand, '/control/command/gear_cmd', qos_profile)

        self.target_speed = 5.0
        self.pid_controller = PIDController(p_gain=1.5, i_gain=0.5, d_gain=0.125, target_vel=self.target_speed)

    def control(self):
        pose_message, velocity_report_message = self.get_latest_messages()

        control_msg = AckermannControlCommand()
        control_msg.stamp = self.get_clock().now().to_msg()
        control_msg.lateral = self.get_lateral_command()
        control_msg.longitudinal = self.get_longitudinal_command(velocity_report_message.longitudinal_velocity if velocity_report_message else 0.0)
        self.control_publisher.publish(control_msg)

        gear_msg = GearCommand()
        gear_msg.stamp = self.get_clock().now().to_msg()
        gear_msg.command = GearCommand.DRIVE
        self.gear_publisher.publish(gear_msg)

    def pose_callback(self, msg):
        self.pose = msg

    def ground_truth_callback(self, msg):
        self.ground_truth = msg

    def velocity_report_callback(self, msg):
        self.velocity_report = msg

    def lookahead_callback(self, msg):
        self.lookahead_point = msg

    def get_latest_messages(self):
        return self.pose, self.velocity_report
    
    def get_lateral_command(self):
        lateral_command = AckermannLateralCommand()
        # if self.pose and self.lookahead_point and self.ground_truth:
        #     lateral_command.steering_tire_angle = self.pure_pursuit_steering_angle()
        #     lateral_command.steering_tire_rotation_rate = 0.0
        # else:
        #     lateral_command.steering_tire_angle = 0.0
        #     lateral_command.steering_tire_rotation_rate = 0.0
        lateral_command.steering_tire_angle = 0.0
        lateral_command.steering_tire_rotation_rate = 0.0
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
    
    def pure_pursuit_steering_angle(self):
        # print("coordinates: ",  self.lookahead_point.look_ahead_point.x, self.lookahead_point.look_ahead_point.y, self.lookahead_point.look_ahead_point.z)
    
        lookahead_x =self.lookahead_point.look_ahead_point.x - self.pose.pose.position.x
        lookahead_y = self.lookahead_point.look_ahead_point.y - self.pose.pose.position.y

        yaw = self.get_yaw_from_pose(self.ground_truth)
        local_x = math.cos(yaw) * lookahead_x + math.sin(yaw) * lookahead_y
        local_y = -math.sin(yaw) * lookahead_x + math.cos(yaw) * lookahead_y

        ld2 = lookahead_x ** 2 + lookahead_y ** 2
        steering_angle = math.atan2(2.0 * local_y, ld2)

        self.get_logger().info(
            f'steering_angle: {steering_angle} :\n'
        )

        return steering_angle

    def get_yaw_from_pose(self, ground_truth):
        orientation = ground_truth.pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def get_gear_command(self):
        gear = GearCommand()
        gear.stamp = self.get_clock().now().to_msg()
        gear.command = GearCommand.DRIVE

def main(args=None):
    rclpy.init(args=args)
    node = VehicleControl()

    while rclpy.ok():
        # rclpy.spin_once(node)
        rclpy.spin_once(node, timeout_sec=0.01)  # Set timeout to 0 to avoid delay
        node.control()
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
