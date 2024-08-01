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
        # print("debug speed: ", target_vel, observed_vel)
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
        self.lookAhead = LookAheadMsg()

        self.control_publisher = self.create_publisher(AckermannControlCommand, '/control/command/control_cmd', qos_profile)
        self.gear_publisher = self.create_publisher(GearCommand, '/control/command/gear_cmd', qos_profile)

        self.pid_controller = PIDController(p_gain=1.5, i_gain=0.5, d_gain=0.125)
        self.wheel_base = 2.75 # meters

        self.previous_steering_angle = 0
        self.steering_gain = 0.2  # Proportional gain for steering
        self.maximum_accel = 4.0

    def pose_callback(self, msg):
        self.pose = msg

    def ground_truth_callback(self, msg):
        self.ground_truth = msg

    def velocity_report_callback(self, msg):
        self.velocity_report = msg

    def lookahead_callback(self, msg):
        self.lookAhead = msg

    def get_latest_messages(self):
        return self.pose, self.velocity_report
    
    def calculate_distance(self, point1, point2):
        return np.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)

    def control(self):

        if not self.velocity_report and not self.lookAhead and not self.pose and not self.ground_truth:
            return

        control_msg = AckermannControlCommand()
        control_msg.stamp = self.get_clock().now().to_msg()

        control_msg.lateral = self.get_lateral_command(self.lookAhead.status.data)
        control_msg.longitudinal = self.get_longitudinal_command(self.lookAhead.status.data)
        self.control_publisher.publish(control_msg)

        gear_msg = GearCommand()
        gear_msg.stamp = self.get_clock().now().to_msg()
        if self.lookAhead.status.data == "Park":
            print("park")
            gear_msg.command = GearCommand.PARK
        else:
            print("drive")
            gear_msg.command = GearCommand.DRIVE
        self.gear_publisher.publish(gear_msg)  

    
    def get_lateral_command(self, status):
        lateral_command = AckermannLateralCommand()
        distance_to_stop = self.calculate_distance(self.lookAhead.stop_point, self.pose.pose.position)
        # or (status == "Decelerate" and distance_to_stop < 10.0)
        if status == "Park":
            print("debug PARK")
            lateral_command.steering_tire_angle = 0.0
            lateral_command.steering_tire_rotation_rate = 0.0
        else:
            if self.pose and self.lookAhead and self.ground_truth:
                steer = self.pure_pursuit_steering_angle()
                lateral_command.steering_tire_angle = steer
                lateral_command.steering_tire_rotation_rate = 0.0
            else:
                lateral_command.steering_tire_angle = 0.0
                lateral_command.steering_tire_rotation_rate = 0.0
        return lateral_command

    def get_longitudinal_command(self, status):

        current_speed = self.velocity_report.longitudinal_velocity if self.velocity_report else 0.0
        target_speed = self.lookAhead.speed_limit
        accel = 0.0

        if status == "Decelerate" or status == "Stop_red" or status == "Stop_amber":
            distance_to_stop = self.calculate_distance(self.lookAhead.stop_point, self.pose.pose.position)
            target_speed = self.calculate_target_speed_for_stop(distance_to_stop, current_speed)

        accel = self.pid_controller.updatePID(current_speed, target_speed)
        if accel > self.maximum_accel:
            accel = self.maximum_accel

        longitudinal_command = LongitudinalCommand()
        longitudinal_command.speed = self.velocity_report.longitudinal_velocity
        longitudinal_command.acceleration = accel

        if status == "Decelerate" or status == "Stop_red" or status == "Stop_amber":
            self.get_logger().info(
            f'speed: {current_speed}\n'
            f'target speed: {target_speed}\n'
            f'stop distance: {self.calculate_distance(self.lookAhead.stop_point, self.pose.pose.position)}\n'
            f'status : {self.lookAhead.status.data}\n'
        )
        else:
            self.get_logger().info(
                f'speed: {current_speed}\n'
                f'target speed: {target_speed}\n'
                f'status : {self.lookAhead.status.data}\n'
            )

        return longitudinal_command
    
    def calculate_target_speed_for_stop(self, distance_to_stop, current_speed):
        # Gradual deceleration based on distance and current speed
        # Using a nonlinear deceleration curve for smoother braking
        return min(self.lookAhead.speed_limit, current_speed * (distance_to_stop / (self.lookAhead.speed_limit * 4))**0.45)

    def filter(self, new_value, previous_value, gain):
        return gain * previous_value + (1 - gain) * new_value

    def pure_pursuit_steering_angle(self):
        # print("coordinates: ",  self.lookAhead.look_ahead_point.x, self.lookAhead.look_ahead_point.y, self.lookAhead.look_ahead_point.z)
    
        lookahead_x =self.lookAhead.look_ahead_point.x - self.pose.pose.position.x
        lookahead_y = self.lookAhead.look_ahead_point.y - self.pose.pose.position.y

        yaw = self.get_yaw_from_pose(self.ground_truth)

        local_x = math.cos(yaw) * lookahead_x + math.sin(yaw) * lookahead_y
        local_y = -math.sin(yaw) * lookahead_x + math.cos(yaw) * lookahead_y

        ld2 = lookahead_x ** 2 + lookahead_y ** 2
        steering_angle = math.atan2(2.0 * local_y * self.wheel_base, ld2)
        steering_angle = self.filter(steering_angle, self.previous_steering_angle, self.steering_gain)
        self.previous_steering_angle = steering_angle

        if steering_angle >= 0:
            self.get_logger().info("Left")
        else:
            self.get_logger().info("Right")
        # self.get_logger().info(
        #     f'steering_angle: {steering_angle}, yaw = {yaw}:\n'
        # )

        return steering_angle
    

    def get_yaw_from_pose(self, ground_truth):
        orientation = ground_truth.pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw


def main(args=None):
    rclpy.init(args=args)
    node = VehicleControl()

    try:
        while rclpy.ok():
            rclpy.spin_once(node)
            rclpy.spin_once(node, timeout_sec=None)# Set timeout to 0 to avoid delay
            node.control()   
    finally:
        node.destroy_node()
        rclpy.shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
