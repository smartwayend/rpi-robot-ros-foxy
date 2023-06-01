#!/usr/bin/env python3

from dataclasses import dataclass
import math
import time
import RPi.GPIO as GPIO
import serial
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType

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


@dataclass
class SerialStatus:
    """Class for different data given by the embedded system"""
    left_ref_speed: float
    right_ref_speed: float
    left_speed:float
    right_speed: float
    left_effort: float
    right_effor: float
    x_pos: float
    y_pos: float
    theta: float
    v: float
    w: float


class RobotControlNode(Node):
    """Simple node for controlling a differential drive robot"""
    
     def init_motor_pins(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.left_front_motor_pin, GPIO.OUT)
        GPIO.setup(self.left_rear_motor_pin, GPIO.OUT)
        GPIO.setup(self.right_front_motor_pin, GPIO.OUT)
        GPIO.setup(self.right_rear_motor_pin, GPIO.OUT)
        self.left_front_motor = GPIO.PWM(self.left_front_motor_pin, 1000)
        self.left_rear_motor = GPIO.PWM(self.left_rear_motor_pin, 1000)
        self.right_front_motor = GPIO.PWM(self.right_front_motor_pin, 1000)
        self.right_rear_motor = GPIO.PWM(self.right_rear_motor_pin, 1000)
        self.left_front_motor.start(0)
        self.left_rear_motor.start(0)
        self.right_front_motor.start(0)
        self.right_rear_motor.start(0)
    
    
    
    
    def __init__(self):
        super().__init__('rpi_robot_node')
        
        
        self.declare_parameter('left_front_motor_pin', 22)
        self.declare_parameter('left_rear_motor_pin', 27)
        self.declare_parameter('right_front_motor_pin', 23)
        self.declare_parameter('right_rear_motor_pin', 24)

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
        
        self.left_front_motor_pin = self.get_parameter('left_front_motor_pin').get_parameter_value().integer_value
        self.left_rear_motor_pin = self.get_parameter('left_rear_motor_pin').get_parameter_value().integer_value
        self.right_front_motor_pin = self.get_parameter('right_front_motor_pin').get_parameter_value().integer_value
        self.right_rear_motor_pin = self.get_parameter('right_rear_motor_pin').get_parameter_value().integer_value
        self.init_motor_pins()
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


    def send_command(self, linear: float, angular: float) -> SerialStatus:
        self.get_logger().debug(f'Data to send: {linear}, {angular}')
        command = f'{linear:.3f},{angular:.3f}/'.encode('UTF-8')
        self.get_logger().debug(f'Sending command: "{command}"')
        self.ser.write(command)
        while self.ser.in_waiting == 0:
            pass

        res = self.ser.read(self.ser.in_waiting).decode('UTF-8')
        self.get_logger().debug(f'data: "{res}", bytes: {len(res)}')

        if res == '0' or len(res) < 79 or len(res) > (79 + 13):
            self.get_logger().warn(f'Bad data: "{res}"')
            return None

        raw_list = res.strip().split('/')[1].split(',')
        
        try:
            values_list = [float(value) for value in raw_list]
        except ValueError as e:
            self.get_logger().warn(f'Bad data: "{res}"')
            return None

        return SerialStatus(*values_list)
    
    
    
    
    def send_command(self, linear: float, angular: float) -> RobotStatus:
        self.get_logger().debug(f'Data to send: {linear}, {angular}')

        # Calculate individual wheel speeds
        left_front_speed = linear - angular
        left_rear_speed = linear - angular
        right_front_speed = linear + angular
        right_rear_speed = linear + angular

        # Apply motor control logic based on wheel speeds
        # You need to implement the control logic for your specific robot hardware

        # Example control logic for controlling motors with PWM on GPIO pins
        left_front_duty_cycle = abs(left_front_speed) * 100
        left_rear_duty_cycle = abs(left_rear_speed) * 100
        right_front_duty_cycle = abs(right_front_speed) * 100
        right_rear_duty_cycle = abs(right_rear_speed) * 100

        GPIO.output(self.left_front_motor_pin, GPIO.HIGH if left_front_speed >= 0 else GPIO.LOW)
        GPIO.output(self.left_rear_motor_pin, GPIO.HIGH if left_rear_speed >= 0 else GPIO.LOW)
        GPIO.output(self.right_front_motor_pin, GPIO.HIGH if right_front_speed >= 0 else GPIO.LOW)
        GPIO.output(self.right_rear_motor_pin, GPIO.HIGH if right_rear_speed >= 0 else GPIO.LOW)

        self.left_front_motor.ChangeDutyCycle(left_front_duty_cycle)
        self.left_rear_motor.ChangeDutyCycle(left_rear_duty_cycle)
        self.right_front_motor.ChangeDutyCycle(right_front_duty_cycle)
        self.right_rear_motor.ChangeDutyCycle(right_rear_duty_cycle)

        # Placeholder values for robot status
        x_pos = 0.0
        y_pos = 0.0
        theta = 0.0
        v = 0.0
        w = 0.0

        return RobotStatus(linear, angular, left_front_speed, right_front_speed, 0.0, 0.0, x_pos, y_pos, theta, v, w)

    
    
    
    
    
    
    
    
    
    def twist_callback(self, twist: Twist):
        self.twist = twist
        self.get_logger().info(f'Twist received: {twist}')


        

def main(args=None):
    rclpy.init(args=args)
    robot_control_node = RobotControlNode()
    rclpy.spin(robot_control_node)

    robot_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
