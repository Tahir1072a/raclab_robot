#!/usr/bin/env python3
import rclpy
import numpy
import math
import sys
import termios

from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

terminal_msg = """
Raclab Robot Position Control
------------------------------------------------------
From the current pose,
x: goal position x (unit: m)
y: goal position y (unit: m)
theta: goal orientation (range: -180 ~ 180, unit: deg)
------------------------------------------------------
"""

def euler_from_quaternion(quat):
        """
        Convert quaternion (w in last place) to euler roll, pitch, yaw.

        quat = [x, y, z, w]
        """
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = numpy.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = numpy.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = numpy.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw



class SelfControl(Node):

    def __init__(self):
        super().__init__("self_controller")

        self.odom = Odometry()
        self.last_pose_x = 0.0
        self.last_pose_y = 0.0
        self.last_pose_theta = 0.0
        self.goal_pose_x = 0.0
        self.goal_pose_y = 0.0
        self.goal_pose_theta = 0.0
        self.step = 1
        self.get_key_state = False
        self.init_odom_state = False

        self.cmd_vel_pub = self.create_publisher(TwistStamped, "raclab_robot_controller/cmd_vel", 10)

        self.odom_sub = self.create_subscription(Odometry, "/raclab_robot_controller/odom", self.odom_callback, 10)

        self.update_timer = self.create_timer(0.010, self.update_callback)  # unit: s

        self.get_logger().info("Raclab Robot position control node has been initialised.")

    def odom_callback(self, msg):
        self.last_pose_x = msg.pose.pose.position.x
        self.last_pose_y = msg.pose.pose.position.y
        _, _, self.last_pose_theta = euler_from_quaternion(msg.pose.pose.orientation)

        self.init_odom_state = True

    def update_callback(self):
         if self.init_odom_state is True:
            self.generate_path()

    def generate_path(self):              
        twist = TwistStamped()

        if self.get_key_state is False:
            input_x, input_y, input_theta = self.get_key()
            self.goal_pose_x = self.last_pose_x + input_x
            self.goal_pose_y = self.last_pose_y + input_y
            self.goal_pose_theta = self.last_pose_theta + input_theta
            self.get_key_state = True

        else:
            
            if self.step == 1:
                path_theta = math.atan2(
                    self.goal_pose_y - self.last_pose_y, 
                    self.goal_pose_x - self.last_pose_x)
                angle = path_theta - self.last_pose_theta
                angular_velocity = 0.3

                twist, self.step = self.turn(angle, angular_velocity, self.step)

            elif self.step == 2:
                distance = math.sqrt(
                    (self.goal_pose_x - self.last_pose_x)** 2 +
                    (self.goal_pose_y - self.last_pose_y)** 2
                )
                linear_velocity = 0.5

                twist, self.step = self.go_straight(distance, linear_velocity, self.step)

            elif self.step == 3:
                angle = self.goal_pose_theta - self.last_pose_theta
                angular_velocity = 0.3
                
                twist, self.step = self.turn(angle, angular_velocity, self.step)

            elif self.step == 4:
                self.step = 1
                self.get_key_state = False

            self.cmd_vel_pub.publish(twist)


    def get_key(self):
        print(terminal_msg)
        input_x = float(input("Input x: "))
        input_y = float(input("Input y: "))
        input_theta = float(input("Input theta: "))

        while input_theta > 180 or input_theta < -180:
            self.get_logger().info("Enter a value for theta between -180 and 180")
            input_theta = float(input("Input Theta: "))
        input_theta = numpy.deg2rad(input_theta)

        settings = termios.tcgetattr(sys.stdin)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        return input_x, input_y, input_theta

    def turn(self, angle, angular_velocity, step):
        twist = TwistStamped()

        if math.fabs(angle) > 0.03:
            if angle >= math.pi:
                twist.twist.angular.z = -angular_velocity
            elif math.pi > angle and angle >= 0:
                twist.twist.angular.z = angular_velocity
            elif 0 > angle and angle >= -math.pi:
                twist.twist.angular.z = -angular_velocity
            elif angle > -math.pi:
                twist.twist.angular.z = angular_velocity

        else:
            step += 1

        return twist, step
    

    def go_straight(self, distance, linear_velocity, step):
        twist = TwistStamped()
        #self.get_logger().info(f"d: {distance}")
        if distance > 0.05:
            twist.twist.linear.x = linear_velocity
        else:
            step += 1
        
        return twist, step


def main():
    rclpy.init()
    node = SelfControl()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()