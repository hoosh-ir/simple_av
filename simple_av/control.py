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
    def __init__(self, p_gain, i_gain, d_gain, delta_t=0.01):
        self.kp = p_gain
        self.ki = i_gain
        self.kd = d_gain
        self.delta_t = delta_t

        self.current_time = time.time()
        self.last_time = self.current_time

        self.integrated_error = 0.0

        self.slidingWindow = deque(maxlen=10) # for storing only the 10 most recent errors

        self.previous_error = 0.0
    
    def updatePID(self, observed_vel, target_vel):
        print("debug 3: ", target_vel, observed_vel)
        error = target_vel - observed_vel
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        self.slidingWindow.append(error)

        self.integrated_error = sum(self.slidingWindow) * delta_time
        derivative = (error - self.previous_error) / delta_time

        P = self.kp * error
        I = self.ki * self.integrated_error
        D = self.kd * derivative
        
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

        self.pid_controller = PIDController(p_gain=1.5, i_gain=0.5, d_gain=0.125)
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
    
    def calculate_distance(self, point1, point2):
        return np.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)
    
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
        target_speed = self.lookahead_point.speed_limit

        if self.lookahead_point.status.data == "continue":
            target_speed = self.lookahead_point.speed_limit
        elif self.lookahead_point.status.data == "stop point":
            distance_to_stop = self.calculate_distance(self.lookahead_point.stop_point, self.pose.pose.position)
            target_speed = self.calculate_target_speed_for_stop(distance_to_stop, current_speed)
        else:
            target_speed = 0.0

        accel = self.pid_controller.updatePID(current_speed, target_speed)

        longitudinal_command = LongitudinalCommand()
        longitudinal_command.speed = self.velocity_report.longitudinal_velocity
        longitudinal_command.acceleration = accel

        self.get_logger().info(
            f'speed: {current_speed} :\n'
            # f'accel : {accel}\n'
            f'stop distance: {self.calculate_distance(self.lookahead_point.stop_point, self.pose.pose.position)} :\n'
            # f'speed_limit : {speed_limit}\n'
            f'status : {self.lookahead_point.status.data}\n'
        )
        return longitudinal_command
    
    def calculate_target_speed_for_stop(self, current_speed, distance_to_stop):
        if distance_to_stop < 1:
            return 0.0  # Immediate stop if very close to the stop point
        else:
            # Gradual deceleration based on distance and current speed
            # Using a nonlinear deceleration curve for smoother braking
            return min(self.lookahead_point.speed_limit, current_speed * (distance_to_stop / 20)**0.5)


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
