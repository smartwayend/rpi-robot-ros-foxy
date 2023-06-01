#!/usr/bin/env python3

import math
import time
import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry


def quaternion_from_euler(roll, pitch, yaw) -> Quaternion:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr

    return q


class RobotControlNode(Node):
    """Simple node for controlling a differential drive robot"""
    def __init__(self):
        super().__init__('rc_car_node')

        self.twist_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.twist_callback,
            10
        )
        self.twist_subscription

        self.odom_publisher = self.create_publisher(
            Odometry,
            'odom',
            10
        )
        time.sleep(0.2)
        self.left_motor_pin1 = 22  # GPIO pin for left motor pin 1
        self.left_motor_pin2 = 27  # GPIO pin for left motor pin 2
        self.right_motor_pin1 = 23  # GPIO pin for right motor pin 1
        self.right_motor_pin2 = 24 # GPIO pin for right motor pin 2

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.left_motor_pin1, GPIO.OUT)
        GPIO.setup(self.left_motor_pin2, GPIO.OUT)
        GPIO.setup(self.right_motor_pin1, GPIO.OUT)
        GPIO.setup(self.right_motor_pin2, GPIO.OUT)

        self.left_motor_pwm = GPIO.PWM(self.left_motor_pin1, 100)
        self.right_motor_pwm = GPIO.PWM(self.right_motor_pin1, 100)

        self.left_motor_pwm.start(0)
        self.right_motor_pwm.start(0)

        self.twist = Twist()
        # set timer
        self.pub_period = 0.04  # 0.02 seconds = 50 hz = pid rate for robot
        self.pub_timer = self.create_timer(self.pub_period, self.pub_callback)
        # tf
        self.tf_broadcaster = TransformBroadcaster(self)

    def pub_callback(self):
        robot_state = self.send_command(self.twist.linear.x, self.twist.angular.z)
        if robot_state is None:
            return

        robot_orientation = quaternion_from_euler(0, 0, robot_state.theta)
        timestamp = self.get_clock().now().to_msg()
        # transforms
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = '/odom'
        t.child_frame_id = '/base_link'
        t.transform.translation.x = robot_state.x_pos
        t.transform.translation.y = robot_state.y_pos
        t.transform.translation.z = 0.0325
        t.transform.rotation = robot_orientation

        # odometry twist
        odom_msg = Odometry()
        odom_msg.header.frame_id = '/odom'
        odom_msg.child_frame_id = '/base_link'
        odom_msg.header.stamp = timestamp
        odom_msg.pose.pose.position.x = robot_state.x_pos
        odom_msg.pose.pose.position.y = robot_state.y_pos
        odom_msg.pose.pose.position.z = 0.325
        odom_msg.pose.pose.orientation = robot_orientation
        odom_msg.twist.twist.linear.x = robot_state.v
        odom_msg.twist.twist.angular.z = robot_state.w

        # broadcast and publish
        self.tf_broadcaster.sendTransform(t)
        self.odom_publisher.publish(odom_msg)

    def send_command(self, linear: float, angular: float):
        self.get_logger().debug(f'Data to send: {linear}, {angular}')

        # Convert linear and angular velocities to motor speeds
        left_speed = linear - angular
        right_speed = linear + angular

        # Adjust motor speeds within the valid range
        left_speed = max(-1.0, min(left_speed, 1.0))
        right_speed = max(-1.0, min(right_speed, 1.0))

        # Map motor speeds to PWM values
        left_pwm = int(left_speed * 100)
        right_pwm = int(right_speed * 100)

        # Set motor directions and speeds using PWM
        if left_speed >= 0:
            GPIO.output(self.left_motor_pin1, GPIO.HIGH)
            GPIO.output(self.left_motor_pin2, GPIO.LOW)
            self.left_motor_pwm.ChangeDutyCycle(left_pwm)
        else:
            GPIO.output(self.left_motor_pin1, GPIO.LOW)
            GPIO.output(self.left_motor_pin2, GPIO.HIGH)
            self.left_motor_pwm.ChangeDutyCycle(100 - left_pwm)

        if right_speed >= 0:
            GPIO.output(self.right_motor_pin1, GPIO.HIGH)
            GPIO.output(self.right_motor_pin2, GPIO.LOW)
            self.right_motor_pwm.ChangeDutyCycle(right_pwm)
        else:
            GPIO.output(self.right_motor_pin1, GPIO.LOW)
            GPIO.output(self.right_motor_pin2, GPIO.HIGH)
            self.right_motor_pwm.ChangeDutyCycle(100 - right_pwm)

    def twist_callback(self, twist: Twist):
        self.twist = twist
        self.get_logger().info(f'Twist received: {twist}')


def main(args=None):
    rclpy.init(args=args)
    #test GPIO 
    '''
    if GPIO.input(left_motor_pin1) == GPIO.HIGH:
      print("Left motor pin 1 is connected")
    else:
      print("Left motor pin 1 is not connected")

    if GPIO.input(right_motor_pin1) == GPIO.HIGH:
      print("Right motor pin 1 is connected")
    else:
      print("Right motor pin 1 is not connected")
    
    '''
    
    robot_control_node = RobotControlNode()
    rclpy.spin(robot_control_node)

    robot_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
