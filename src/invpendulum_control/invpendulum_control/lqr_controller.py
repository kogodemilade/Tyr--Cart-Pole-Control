#!/usr/bin/env python3

import rclpy, numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped, TransformStamped
from sensor_msgs.msg import JointState
from rclpy.time import Time
from rclpy.constants import S_TO_NS
import math
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
import random
import numpy as np


class Controller(Node):
    def __init__(self):
        super().__init__("pendulum_controller")
        self.get_logger().info("Using LQR")

        self.declare_parameter("wheel_radius", 0.01) #Check and change
        self.declare_parameter("pendulum_length", 0.566)
        self.declare_parameter("kp", 12.0)
        self.declare_parameter("kd", 1.0)
        self.declare_parameter("ki", 4.0)
        # self.declare_parameter("cart mass", 2)

        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.pendulum_length = self.get_parameter("pendulum_length").get_parameter_value().double_value

        # self.position_gain = -14.1421
        # self.angular_vel_gain = -19.7660
        # self.position_gain = 152.8992
        # self.vel_gain = 50.2005

        self.optimal_gain_vec = np.array([-6.3246, -10.3021, 103.5373, 34.0491])
        
        
        self.prev_time = self.get_clock().now()
        self.pole_angle = 0.0
        self.pole_velocity = 0.0

        self.cart_position = 0.0
        self.cart_velocity = 0.0

        self.prev_pole_angle = 0.0
        self.cumulative_error = 0.0

        self.force_gain = 10e0
        self.signal_gain = 10e0
        self.pole_mass = 0.48
        self.swing_gain = 10e2
        
        # self.wheel_cmd_pub = self.create_publisher(Float64MultiArray, "velocity_controller/commands", 10)
        self.swing_up_pub = self.create_publisher(Float64MultiArray, "effort_controller/commands", 100)

        self.get_logger().info('Created cart command publisher node')
        self.joint_sub = self.create_subscription(JointState, "joint_states", self.jointCallback, 100)
        self.get_logger().info('Created joint states subscriber node')

        self.br = TransformBroadcaster(self)
        self.transform = TransformStamped()
        self.transform.header.frame_id = "odom"
        self.transform.child_frame_id = "base_footprint"
        self.i = 0
        self.start_time = self.get_clock().now()


    
    def jointCallback(self, msg):
        poison = random.random()*0.00
        # poison = 0
        cart_index= msg.name.index("cart_joint")
        cart_position = msg.position[cart_index]+poison
        self.cart_position = cart_position

        self.cart_velocity = msg.velocity[cart_index]+poison

        # AngularVel = msg.velocity[cart_index]
        # self.cart_velocity = self.wheel_radius*AngularVel 
        self.command = Float64MultiArray()


        pole_index= msg.name.index("pendulum_shaft_joint")
        self.pole_angle = msg.position[pole_index]+poison
        self.pole_velocity = msg.velocity[pole_index]+poison

        # self.pole
        # self.sign = self
        # if self.pole_angle > 3.1415:
        #     self.pole_angle += -2*3.1415926


        current_time = Time.from_msg(msg.header.stamp)
        dt = current_time - self.prev_time
        dt = dt.nanoseconds/ S_TO_NS
        self.prev_time = current_time

        if dt == 0:
            return

        # if (self.get_clock().now() - self.start_time).nanoseconds/ S_TO_NS < 10:
        #     self.command.data = [0.0]
        #     self.temp_pendulum_pub.publish(self.command)
        #     return
        

        if abs(self.pole_angle) < 0.3: #LQR controller
            
            # derivative = (self.pole_angle - self.prev_pole_angle)/dt
            derivative = self.pole_velocity
            # self.prev_pole_angle = self.pole_angle
            # self.cumulative_error += self.pole_angle*dt
            
            # proportional_ = self.kp*self.pole_angle
            # integral_ = np.clip(self.ki*self.cumulative_error, -5.0, 5.0)
            # derivative_ = self.kd*derivative

            # angle_correction = self.pole_angle*self.angle_gain
            # ang_vel_correction = self.pole_velocity*self.angular_vel_gain
            # pos_correction = self.cart_position*self.position_gain
            # velocity_correction = self.cart_velocity*self.vel_gain
            current_states_vec = np.array([self.cart_position, self.cart_velocity, self.pole_angle, self.pole_velocity])
            correction = self.optimal_gain_vec.dot(current_states_vec)
            signal = correction * 1

            final_signal = -signal
            self.command.data = [final_signal]
            self.swing_up_pub.publish(self.command)
            # self.get_logger().info(f"adjusting signal:  {final_signal}", )
            # self.get_logger().info(f"proportional:  {proportional_}", )
            # self.get_logger().info(f"integral:  {integral_}", )
            # self.get_logger().info(f"der:  {derivative_}", )
            # self.get_logger().info(f"pole angle:  {self.pole_angle}", )
            # self.get_logger().info(f"kp:  {self.kp}", )
            # self.get_logger().info(f"kd:  {self.kd}", )
            # self.get_logger().info(f"ki:  {self.ki}", )


        
        else: #Swing up controller using energy shaping

            if self.cart_velocity < 0.05: #inject velocity 
                d_list = [1, -1]
                discriminant = random.randint(0, 1)
                self.command.data = [15000 * d_list[discriminant]]
                self.swing_up_pub.publish(self.command)

            Energy = 0.5 * self.pole_mass * self.pendulum_length * self.pendulum_length * self.pole_velocity * self.pole_velocity 
            + self.pole_mass * 9.81 * self.pendulum_length * (math.cos(self.pole_angle) - 1.0)

            force = -self.swing_gain *Energy *self.pole_velocity*math.cos(self.pole_angle)
            self.command.data = [force]
            self.swing_up_pub.publish(self.command)
            # self.get_logger().info(f"signal: {force}")r
        

        if self.i % 400 == 0:
            self.i = 0
            self.get_logger().info(f"pole angle:  {self.pole_angle}", )
            self.get_logger().info(f"cart pos:  {self.cart_position}", )


            self.get_logger().info(f"adjusting signal:  {self.command.data}", )

            
        self.i += 1


def main():
    rclpy.init()
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
