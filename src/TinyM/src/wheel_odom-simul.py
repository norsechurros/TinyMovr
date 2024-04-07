#!/usr/bin/env python3.10

import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Quaternion, Point
import math
import tf

class WheelOdometry:

    def __init__(self):
        rospy.init_node('wheel_odometry', anonymous=True)
        self.enc_vel_tm1 = 0.0
        self.enc_vel_tm2 = 0.0
        self.enc_pos_tm1 = 0.0
        self.enc_pos_tm2 = 0.0
        
        self.prev_time = rospy.Time.now()
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)  # Publish to /odom
        self.odom_pub_position = rospy.Publisher('/odom_position', Odometry, queue_size=10)  # Publish odometry from position estimates
        self.avg_vel_pub = rospy.Publisher("/avg_vel", Float64, queue_size=1)
        self.avg_angle_pub = rospy.Publisher("/avg_angle", Float64, queue_size=1)
        
        self.odom = Odometry()
        self.odom.header.frame_id = "odom"
        self.odom.child_frame_id = "base_link"
        self.pose = Pose()
        self.twist = Twist()
        self.vels = []
        self.angles = []

        self.wheel_distance = 0.76 #0.76  # Distance between wheels (assumed value)
        self.wheel_radius = 0.19  # Radius of each wheel (assumed value)
        
        self.theta = 0.0
        self.alpha = 0.2  # Adjust as needed for your filter
        
        self.filtered_linear_vel = 0.0
        self.filtered_angular_vel = 0.0

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
        self.vel1 = self.enc_vel_tm1 * 0.0010362
        self.vel2 = self.enc_vel_tm2 * 0.0010362

        # Calculate linear and angular velocity from encoder velocity estimates
        linear_vel = (self.vel2 - self.vel1) / 2.0
        angular_vel = (-self.vel1 - self.vel2) / self.wheel_distance

        # Apply low-pass filter using exponential moving average (EMA)
        self.filtered_linear_vel = (1 - self.alpha) * self.filtered_linear_vel + self.alpha * linear_vel
        self.filtered_angular_vel = (1 - self.alpha) * self.filtered_angular_vel + self.alpha * angular_vel

        # Calculate lateral velocity
        lateral_vel = self.filtered_angular_vel * self.wheel_distance / 2

        # Update pose using position estimates only
        self.pose.position.x = (self.enc_pos_tm1 + self.enc_pos_tm2) / 2.0
        self.pose.position.y = lateral_vel * dt
        self.pose.position.z = 0.0  # Assuming no vertical movement

        # Update orientation (quaternion)
        self.pose.orientation = Quaternion(0, 0, math.sin(self.theta / 2), math.cos(self.theta / 2))

        # Update twist
        self.twist.linear.x = linear_vel
        self.twist.linear.y = lateral_vel
        self.twist.angular.z = 0.0  # Assuming no angular velocity

        # Update header
        self.odom.header.stamp = current_time

        self.prev_time = current_time

    def calculate_wheel_odom_pos(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.prev_time).to_sec()
        
        # Calculate linear velocity from position estimates
        linear_vel = (self.enc_pos_tm1 - self.enc_pos_tm2) / dt
        
        # Calculate lateral velocity from position estimates
        lateral_vel = (self.enc_pos_tm1 + self.enc_pos_tm2) / (2 * dt)
        
        # Update pose using position estimates only
        self.pose.position.x = (self.enc_pos_tm1 + self.enc_pos_tm2) / 2.0
        self.pose.position.y = lateral_vel * dt
        self.pose.position.z = 0.0  # Assuming no vertical movement
        
        # Update orientation (quaternion)
        self.pose.orientation = Quaternion(0, 0, math.sin(self.theta / 2), math.cos(self.theta / 2))
        
        # Update twist
        self.twist.linear.x = linear_vel
        self.twist.linear.y = lateral_vel
        self.twist.angular.z = 0.0  # Assuming no angular velocity
        
        # Update header
        self.odom.header.stamp = current_time
        self.prev_time = current_time

    def publish_wheel_odometry(self):
        self.calculate_wheel_odometry()
        self.odom.pose.pose = self.pose
        self.odom.twist.twist = self.twist
        self.odom_pub.publish(self.odom)

        # Publish odometry calculated from position estimates
        self.calculate_wheel_odom_pos()
        odom_position = Odometry()
        odom_position.header = self.odom.header
        odom_position.pose.pose = self.pose
        odom_position.twist.twist = self.twist
        self.odom_pub_position.publish(odom_position)

    def main(self):
        rospy.loginfo("Wheel Odometry Node started.")
        rate = rospy.Rate(50)  # 50 Hz
        br = tf.TransformBroadcaster()
        while not rospy.is_shutdown():
            br.sendTransform((self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, self.odom.pose.pose.position.z), 
                             (self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w), 
                             rospy.Time.now(), "base_link", "odom")
            self.publish_wheel_odometry()
            rate.sleep()

if __name__ == '__main__':
    try:
        wheel_odometry = WheelOdometry()
        wheel_odometry.main()
    except rospy.ROSInterruptException:
        pass
