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
import numpy as np


class PIDController:
    def __init__(self):

        self.kp_s = 1.5
        self.ki_s = 0.5
        self.kd_s = 0.125

        self.kp_d = 1
        self.ki_d = 0.1
        self.kd_d = 0.2

        self.current_time = time.time()
        self.last_time = self.current_time

        self.integrated_error_s = 0.0
        self.integrated_error_d = 0.0

        self.slidingWindow_s = deque(maxlen=10) # for storing only the 10 most recent errors
        self.slidingWindow_d = deque(maxlen=10) # for storing only the 10 most recent errors

        self.previous_error_s = 0.0
        self.previous_error_d = 0.0

    def calculate_distance(self, point1, point2):
        """
        Calculate the Euclidean distance between two points.
        Args:
            point1 (dict): The first point with 'x', 'y', 'z' coordinates.
            point2 (dict): The second point with 'x', 'y', 'z' coordinates.
        Returns:
            float: The Euclidean distance between the two points.
        """
        return np.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)
    
    def control_by_distance(self, stop_point, vehicle_pose):
        
        
        error = self.calculate_distance(stop_point, vehicle_pose)
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        self.slidingWindow_d.append(error)

        self.integrated_error_d = sum(self.slidingWindow_d) * delta_time
        derivative = (error - self.previous_error_d) / delta_time

        P = self.kp_d * error
        I = self.ki_d * self.integrated_error_d
        D = self.kd_d * derivative

        self.last_time = self.current_time
        self.previous_error_d = error

        return P + I + D

    def control_by_speed(self, observed_vel):
        error = self.target_vel - observed_vel
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        self.slidingWindow_s.append(error)

        self.integrated_error_s = sum(self.slidingWindow_s) * delta_time
        derivative = (error - self.previous_error_s) / delta_time

        P = self.kp_s * error
        I = self.ki_s * self.integrated_error_s
        D = self.kd_s * derivative
        
        acc_cmd = P + I + D

        self.last_time = self.current_time
        self.previous_error_s = error

        return acc_cmd

    def updatePID(self, observed_vel, stop_point, speed_limit, status, vehicle_pose):
        self.target_vel = speed_limit
        if status == 'stop point':
           print("debug 0: distance control")
           acc_cmd =  self.control_by_distance(stop_point, vehicle_pose)
        else:
            print("debug 0: speed control")
            acc_cmd = self.control_by_speed(observed_vel)

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

        self.pid_controller = PIDController()
        self.wheel_base = 2.75 # meters

    def control(self):

        if not self.velocity_report and not self.lookahead_point and not self.pose and not self.ground_truth:
            return

        control_msg = AckermannControlCommand()
        control_msg.stamp = self.get_clock().now().to_msg()
        control_msg.lateral = self.get_lateral_command()
        control_msg.longitudinal = self.get_longitudinal_command()
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
        if self.pose and self.lookahead_point and self.ground_truth:
            steer = self.pure_pursuit_steering_angle()
            # steer1 = self.pure_pursuit_steering_angle1()
            # if steer == steer1:
            #     print("same: ", steer, " ---- ", steer1)
            # else:
            #     print("different: ", steer, " ---- ", steer1)
            lateral_command.steering_tire_angle = steer
            lateral_command.steering_tire_rotation_rate = 0.0
        else:
            lateral_command.steering_tire_angle = 0.0
            lateral_command.steering_tire_rotation_rate = 0.0

        return lateral_command

    def get_longitudinal_command(self):
        current_speed = self.velocity_report.longitudinal_velocity if self.velocity_report else 0.0

        stop_point = self.lookahead_point.stop_point
        speed_limit = self.lookahead_point.speed_limit
        status = self.lookahead_point.status.data

        longitudinal_command = LongitudinalCommand()
        longitudinal_command.speed = self.velocity_report.longitudinal_velocity

        accel = self.pid_controller.updatePID(current_speed, stop_point, speed_limit, status, self.pose)
        longitudinal_command.acceleration = accel
        self.get_logger().info(
            f'speed: {current_speed} :\n'
            f'accel : {accel}\n'
            f'brake_line: {stop_point} :\n'
            f'speed_limit : {speed_limit}\n'
            f'status : {status}\n'
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
        steering_angle = math.atan2(2.0 * local_y * self.wheel_base, ld2)
        
        # self.get_logger().info(
        #     f'steering_angle: {steering_angle} :\n'
        # )

        return steering_angle
    
    def pure_pursuit_steering_angle1(self):

        lookahead_x =self.lookahead_point.look_ahead_point.x - self.pose.pose.position.x
        lookahead_y = self.lookahead_point.look_ahead_point.y - self.pose.pose.position.y

        yaw = self.get_yaw_from_pose(self.ground_truth)
        ld2 = lookahead_x ** 2 + lookahead_y ** 2

        # calculate control input
        alpha = np.arctan2(lookahead_y, lookahead_x) - yaw # vehicle heading angle error
        steering_angle = np.arctan2(2.0 * self.wheel_base * np.sin(alpha) / ld2, 1.0) # given from geometric relationship

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
