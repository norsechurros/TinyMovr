#!/usr/bin/env python3.10

import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Quaternion
import math
from math import e

class WheelOdometry:

    def __init__(self):
        rospy.init_node('wheel_odometry', anonymous=True)
        self.enc_vel_tm1 = 0.0
        self.enc_vel_tm2 = 0.0
        self.enc_pos_tm1 = 0.0
        self.enc_pos_tm2 = 0.0
        
        self.prev_time = rospy.Time.now()
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)  # Publish to /odom
        self.avg_vel_pub = rospy.Publisher("/avg_vel", Float64, queue_size=1)
        self.avg_angle_pub = rospy.Publisher("/avg_angle", Float64, queue_size=1)
        
        self.odom = Odometry()
        self.odom.header.frame_id = "odom"
        self.odom.child_frame_id = "base_footprint"
        self.pose = Pose()
        self.twist = Twist()
        self.vels = []
        self.angles = []

        self.wheel_distance = 0.68#0.76  # Distance between wheels (assumed value)
        self.wheel_radius = 0.19  # Radius of each wheel (assumed value)
        self.theta = 0.0
        self.prev_coor = 0
        self.linear_displacement = 0
        rospy.Subscriber('encoder_vel_tm1', Float64, self.enc_tm1_callback)
        rospy.Subscriber('encoder_vel_tm2', Float64, self.enc_tm2_callback)
        rospy.Subscriber('encoder_pos_tm1', Float64, self.enc_pos_tm1_callback)
        rospy.Subscriber('encoder_pos_tm2', Float64, self.enc_pos_tm2_callback)

    def enc_tm1_callback(self, data):
        self.enc_vel_tm1 = data.data

    def enc_tm2_callback(self, data):
        self.enc_vel_tm2 = data.data
        
    def enc_pos_tm1_callback(self, data):
        self.enc_pos_tm1 = data.data
        
    def enc_pos_tm2_callback(self, data):
        self.enc_pos_tm2 = data.data

    def calculate_wheel_odometry(self):
      
        current_time = rospy.Time.now()
        dt = (current_time - self.prev_time).to_sec()

        # Use position values instead of velocity
        pos1 = self.enc_pos_tm1 * 0.0010362
        pos2 = self.enc_pos_tm2 * 0.0010362

        # Calculate linear and angular displacement
        linear_displacement = (pos2 - pos1) / 2.0
        print(linear_displacement)
        angular_displacement = (pos2 + pos1) / self.wheel_distance

        # Calculate lateral displacement
        lateral_displacement = angular_displacement * self.wheel_distance / 2

        if (self.prev_coor == 0 + 0j):
            self.prev_coor = linear_displacement * e ** (1j * self.theta)
            self.pose.position.x = (linear_displacement * math.cos(self.theta))
            self.pose.position.y = (linear_displacement * math.sin(self.theta)) + lateral_displacement
            self.pose.position.z = 0 
        else:
            self.pose.position.x = self.prev_coor.real + (linear_displacement - self.linear_displacement) * math.cos(self.theta)
            self.pose.position.y = self.prev_coor.imag + (linear_displacement - self.linear_displacement) * math.sin(self.theta) + lateral_displacement
            self.pose.position.z = 0
            
        self.linear_displacement = linear_displacement
        # Update pose
         # Assuming no vertical movement

        # Update orientation (quaternion)
        self.theta = angular_displacement 
        self.pose.orientation = Quaternion(0, 0, math.sin(self.theta/2), math.cos(self.theta/2))

        # Update twist
        self.twist.linear.x = linear_displacement / dt  # Calculate linear velocity
        self.twist.linear.y = lateral_displacement / dt  # Calculate lateral velocity
        self.twist.angular.z = angular_displacement / dt  # Calculate angular velocity

        # Update header
        self.odom.header.stamp = current_time

        self.prev_time = current_time
        
    def publish_wheel_odometry(self):
        self.calculate_wheel_odometry()
        self.odom.pose.pose = self.pose
        self.odom.twist.twist = self.twist
        self.odom_pub.publish(self.odom)

    def main(self):
        rospy.loginfo("Wheel Odometry Node started.")
        rate = rospy.Rate(50)  # 50 Hz
        while not rospy.is_shutdown():
            self.publish_wheel_odometry()
            rate.sleep()

if __name__ == '__main__':
    try:
        wheel_odometry = WheelOdometry()
        wheel_odometry.main()
    except rospy.ROSInterruptException:
        pass
