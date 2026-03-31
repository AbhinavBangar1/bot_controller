#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import math

class BotController(Node) :
    def __init__(self):
        super().__init__("bot_controller")

        ## Robo Geo
        self.wheel_rad = 0.05
        self.wheel_base = 0.20
        self.dt = 0.02  # llop time

        ## Tuning values
        self.kp = 0.5
        self.ki = 0.0
        self.kd = 0.0 
        self.left_integral = 0.0
        self.right_integral = 0.0
        self.left_prev_error = 0.0
        self.right_prev_error = 0.0

        ## Velocity States
        self.target_left_vel = 0.0
        self.target_right_vel = 0.0
        self.curr_left_vel = 0.0
        self.curr_right_vel = 0.0

        ## Robot States
        self.x = 0.0
        self.y = 0.0 
        self.theta = 0.0

        ## Subscritpions
        self.create_subscription(Twist , '/cmd_vel' , self.cmd_cb , 10)
        self.create_subscription(JointState , '/joint_states' , self.joint_cb , 10)

        ## Publishers
        self.left_pub = self.create_publisher(Float64 , '/left_wheel_velocity_controller/command' , 10)
        self.right_pub = self.create_publisher(Float64 , '/right_wheel_velocity_controller/command' , 10)

        ## timer loop
        self.timer = self.create_timer(self.dt ,self.control_loop)

    def cmd_cb(self,msg):
        v = msg.linear.x
        w = msg.angular.z
        c = w * self.wheel_base / 2

        self.target_left_vel = (v - c) / self.wheel_rad
        self.target_right_vel = (v + c) / self.wheel_rad

    def joint_cb(self,msg):
        try :
            left = msg.name.index("left_wheel_joint")
            right = msg.name.index("right_wheel_joint")
            self.curr_left_vel = msg.velocity[left]
            self.curr_right_vel = msg.velocity[right]
        except ValueError :
            # self.get_logger().info("Joint not found in msg")
            pass

    def pid (self , current , target , integral , prev_error):
        error = target - current
        integral += error * self.dt
        der = (error - prev_error) / self.dt
        out = self.kp * error + self.ki * integral + self.kd * der
        return out , integral , error
        
    def control_loop(self) :
        left_pid , self.left_integral , self.left_prev_error = self.pid(self.curr_left_vel , self.target_left_vel , self.left_integral ,self.left_prev_error)
        right_pid , self.right_integral , self.right_prev_error = self.pid(self.curr_right_vel , self.target_right_vel , self.right_integral ,self.right_prev_error)

        left_msg = Float64()
        right_msg = Float64()
        left_msg.data = left_pid
        right_msg.data = right_pid
        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)

        ## Odometry calculation
        v_l = self.curr_left_vel * self.wheel_rad 
        v_r = self.curr_right_vel * self.wheel_rad
        v = (v_l + v_r) / 2 ## Central linear velocity
        w = (v_r - v_l) / self.wheel_base ## Central angular velocity
        self.x = self.x + v*math.cos(self.theta)*self.dt
        self.y = self.y + v*math.sin(self.theta)*self.dt
        self.theta = self.theta + w*self.dt
        self.get_logger().info(f"Odometry : x = {self.x} , y = {self.y} , theta = {self.theta}")


def main(args=None):
    rclpy.init()
    node = BotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
