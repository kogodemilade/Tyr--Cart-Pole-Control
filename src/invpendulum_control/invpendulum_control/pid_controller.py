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


class Controller(Node):
    def __init__(self):
        super().__init__("pendulum_controller")

        self.declare_parameter("wheel_radius", 0.01) #Check and change
        self.declare_parameter("pendulum_length", 0.566) #Check and change
        self.declare_parameter("kp", 12.0)
        self.declare_parameter("kd", 1.0)
        self.declare_parameter("ki", 4.0)

        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.pendulum_length = self.get_parameter("pendulum_length").get_parameter_value().double_value

        self.kp = self.get_parameter("kp").get_parameter_value().double_value
        self.kd = self.get_parameter("kd").get_parameter_value().double_value
        self.ki = self.get_parameter("ki").get_parameter_value().double_value

        self.signal_gain = 10
        
        self.prev_time = self.get_clock().now()
        self.pole_angle = 0.0
        self.pole_velocity = 0.0

        self.cart_position = 0.0
        self.cart_velocity = 0.0

        self.prev_pole_angle = 0.0
        self.cumulative_error = 0.0

        self.force_gain = 10e5

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
        poison = random.random()*0.005
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
        

        if abs(self.pole_angle) < 10: #PID controller
            self.get_logger().info("Using pid")
            # derivative = (self.pole_angle - self.prev_pole_angle)/dt
            derivative = self.pole_velocity
            # self.prev_pole_angle = self.pole_angle
            self.cumulative_error += self.pole_angle*dt
            
            proportional_ = self.kp*self.pole_angle
            integral_ = np.clip(self.ki*self.cumulative_error, -5.0, 5.0)
            derivative_ = self.kd*derivative

            signal = proportional_+integral_+derivative_

            final_signal = -signal*self.signal_gain*self.force_gain
            self.command.data = [final_signal]
            self.swing_up_pub.publish(self.command)
            self.get_logger().info(f"adjusting signal:  {final_signal}", )
            self.get_logger().info(f"proportional:  {proportional_}", )
            self.get_logger().info(f"integral:  {integral_}", )
            self.get_logger().info(f"der:  {derivative_}", )
            self.get_logger().info(f"pole angle:  {self.pole_angle}", )
            # self.get_logger().info(f"kp:  {self.kp}", )
            # self.get_logger().info(f"kd:  {self.kd}", )
            # self.get_logger().info(f"ki:  {self.ki}", )


        
        else: #Swing up controller using energy shaping
            Energy = (0.5*(self.pendulum_length**2)*(self.pole_velocity**2)) + (9.81*self.pendulum_length*(1- math.cos(self.pole_angle)))
            force = -self.force_gain *Energy *self.pole_velocity*math.cos(self.pole_angle)
            self.command.data = [force]
            self.swing_up_pub.publish(self.command)
            # self.get_logger().info(f"signal: {force}")r
        

        if self.i % 400 == 0:
            self.i = 0
            self.get_logger().info(f"pole angle:  {self.pole_angle}", )

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
