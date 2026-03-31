#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState , Imu
from std_msgs.msg import Float64
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster # transform tree

class BotController(Node) :
    def __init__(self):
        super().__init__("bot_controller")

        self.odom_msg = Odometry()

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

        ## Imu
        self.imu_yaw = 0.0

        ## Fusion Factors
        self.alpha = 0.98

        ## EKF(Extended kalman Filter)

        ## Subscritpions
        self.create_subscription(Twist , '/cmd_vel' , self.cmd_cb , 10)
        self.create_subscription(JointState , '/joint_states' , self.joint_cb , 10)
        self.create_subscription(Imu , '/imu/data' , self.imu_cb , 10)

        ## Publishers
        self.left_pub = self.create_publisher(Float64 , '/left_wheel_velocity_controller/command' , 10)
        self.right_pub = self.create_publisher(Float64 , '/right_wheel_velocity_controller/command' , 10)
        self.odom_pub = self.create_publisher(Odometry , '/odom' , 10)

        ## Tf
        self.tf_broadcaster = TransformBroadcaster(self)

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

    def imu_cb (self,msg):
        qx, qy , qz , qw = msg.orientation.x , msg.orientation.y , msg.orientation.z , msg.orientation.w
        yaw = atan2(2*qw*qz , 1-2*qz*qz)
        self.imu_yaw = yaw

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
        self.theta = self.alpha * self.theta + (1 - self.alpha) * self.imu_yaw

        odom_msg = self.odom_msg
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2)
        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.angular.z = w

        self.odom_pub.publish(odom_msg)
        self.get_logger().info(f"Odometry : x = {self.x} , y = {self.y} , theta = {self.theta}")

        ## Transform Tree Calculations
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_link"
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = math.sin(self.theta / 2)
        tf_msg.transform.rotation.w = math.cos(self.theta / 2)
        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init()
    node = BotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
