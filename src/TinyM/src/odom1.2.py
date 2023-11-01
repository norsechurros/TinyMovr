#!/usr/bin/env python3.10

import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Quaternion, Vector3
import math

class WheelOdometry:

    def __init__(self):
        rospy.init_node('wheel_odometry', anonymous=True)
        self.enc_pos_tm1 = 0.0
        self.enc_pos_tm2 = 0.0
        self.prev_enc_pos_tm1 = 0.0
        self.prev_enc_pos_tm2 = 0.0

        self.prev_time = rospy.Time.now()
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

        self.odom = Odometry()
        self.odom.header.frame_id = "odom"
        self.odom.child_frame_id = "base_footprint"
        self.pose = Pose()
        self.twist = Twist()

        self.wheel_distance = 0.76
        self.wheel_radius = 0.19

        rospy.Subscriber('encoder_pos_tm1', Float64, self.enc_pos_tm1_callback)
        rospy.Subscriber('encoder_pos_tm2', Float64, self.enc_pos_tm2_callback)

    def enc_pos_tm1_callback(self, data):
        self.enc_pos_tm1 = data.data

    def enc_pos_tm2_callback(self, data):
        self.enc_pos_tm2 = data.data

    def calculate_wheel_odometry(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.prev_time).to_sec()

        delta_enc_pos_tm1 = self.enc_pos_tm1 - self.prev_enc_pos_tm1
        delta_enc_pos_tm2 = self.enc_pos_tm2 - self.prev_enc_pos_tm2

        linear_vel = ((delta_enc_pos_tm1 + delta_enc_pos_tm2) / 2.0) * self.wheel_radius / dt
        angular_vel = ((delta_enc_pos_tm2 - delta_enc_pos_tm1) / self.wheel_distance) * self.wheel_radius / dt

        self.pose.position.x += (linear_vel * dt)
        self.pose.position.y = 0  # Assuming no lateral movement
        self.pose.position.z = 0  # Assuming no vertical movement

        # Update orientation (quaternion) properly
        delta_theta = angular_vel * dt
        quaternion = self.pose.orientation
        q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        half_delta_theta = 0.5 * delta_theta
        q_delta = [math.sin(half_delta_theta), 0, 0, math.cos(half_delta_theta)]
        updated_q = [a * b for a, b in zip(q, q_delta)]
        self.pose.orientation = Quaternion(*updated_q)

        # Update twist with proper angular velocities
        self.twist.linear = Vector3(linear_vel, 0, 0)  # Assuming no lateral velocity
        self.twist.angular = Vector3(angular_vel, 0, 0)  # Update angular velocity in x and y directions

        # Update header
        self.odom.header.stamp = current_time

        self.prev_enc_pos_tm1 = self.enc_pos_tm1
        self.prev_enc_pos_tm2 = self.enc_pos_tm2
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
