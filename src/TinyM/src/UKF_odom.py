#!/usr/bin/env python3.10

import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Quaternion
import math
import numpy as np
from filterpy.kalman import UnscentedKalmanFilter
from filterpy.kalman import MerweScaledSigmaPoints

class WheelOdometry:

    def __init__(self):
        rospy.init_node('wheel_odometry', anonymous=True)
        self.enc_vel_tm1 = 0.0
        self.enc_vel_tm2 = 0.0
        
        self.prev_time = rospy.Time.now()
        self.odom_pub = rospy.Publisher('/Odom', Odometry, queue_size=10)  # Publish to /Odom
        self.ukf_odom_pub = rospy.Publisher('/UKFOdom', Odometry, queue_size=10)  # Publish to /UKFOdom
        self.avg_vel_pub = rospy.Publisher("/avg_vel", Float64, queue_size=1)
        self.avg_angle_pub = rospy.Publisher("/avg_angle", Float64, queue_size=1)
        
        self.odom = Odometry()
        self.odom.header.frame_id = "odom"
        self.odom.child_frame_id = "base_link"
        self.ukf_odom = Odometry()
        self.ukf_odom.header.frame_id = "odom"
        self.ukf_odom.child_frame_id = "base_link"
        
        self.pose = Pose()
        self.twist = Twist()
        self.ukf_pose = Pose()
        self.ukf_twist = Twist()
        self.vels = []
        self.angles = []

        self.wheel_distance = 0.76 #0.76  # Distance between wheels (assumed value)
        self.wheel_radius = 0.19  # Radius of each wheel (assumed value)
        self.theta = 0.0
        
        # Initialize Unscented Kalman filter
        points = MerweScaledSigmaPoints(n=3, alpha=0.1, beta=2., kappa=1.)
        self.ukf = UnscentedKalmanFilter(dim_x=3, dim_z=1, dt=0.1, points=points)
        self.ukf.x = np.array([0., 0., 0.])  # Initial state estimate
        self.ukf.P *= 1000  # Covariance matrix
        self.ukf.R = 0.01  # Measurement noise covariance
        self.ukf.Q = np.eye(3) * 0.001  # Process noise covariance

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

        # Predict step
        self.ukf.predict(dt=dt)

        # Update step
        self.ukf.update(z=self.enc_vel_tm1)

        # Get the updated state estimate
        x = self.ukf.x

        # Extract updated values
        linear_vel = x[0]
        angular_vel = x[1]
        self.theta = x[2]

        # Update lateral velocity
        lateral_vel = angular_vel * self.wheel_distance / 2

        # Update pose
        self.ukf_pose.position.x += (linear_vel * dt * math.cos(self.theta))
        self.ukf_pose.position.y += (linear_vel * dt * math.sin(self.theta)) #+ lateral_vel * dt 
        self.ukf_pose.position.z = 0  # Assuming no vertical movement

        # Update orientation (quaternion)
        self.ukf_pose.orientation = Quaternion(0, 0, math.sin(self.theta/2), math.cos(self.theta/2))

        # Update twist
        self.ukf_twist.linear.x = linear_vel
        self.ukf_twist.linear.y = lateral_vel
        self.ukf_twist.angular.z = angular_vel

        # Update header
        self.ukf_odom.header.stamp = current_time
        self.ukf_odom.pose.pose = self.ukf_pose
        self.ukf_odom.twist.twist = self.ukf_twist
        self.ukf_odom_pub.publish(self.ukf_odom)
        
        # Update pose for non-Kalman filtered data
        self.pose.position.x += (linear_vel * dt * math.cos(self.theta))
        self.pose.position.y += (linear_vel * dt * math.sin(self.theta)) #+ lateral_vel * dt 
        self.pose.position.z = 0  # Assuming no vertical movement

        # Update orientation (quaternion) for non-Kalman filtered data
        self.pose.orientation = Quaternion(0, 0, math.sin(self.theta/2), math.cos(self.theta/2))

        # Update twist for non-Kalman filtered data
        self.twist.linear.x = linear_vel
        self.twist.linear.y = lateral_vel
        self.twist.angular.z = angular_vel

        # Update header for non-Kalman filtered data
        self.odom.header.stamp = current_time
        self.odom.pose.pose = self.pose
        self.odom.twist.twist = self.twist
        self.odom_pub.publish(self.odom)

        self.prev_time = current_time

    def main(self):
        rospy.loginfo("Wheel Odometry Node started.")
        rate = rospy.Rate(50)  # 50 Hz
        while not rospy.is_shutdown():
            self.calculate_wheel_odometry()
            rate.sleep()

if __name__ == '__main__':
    try:
        wheel_odometry = WheelOdometry()
        wheel_odometry.main()
    except rospy.ROSInterruptException:
        pass
