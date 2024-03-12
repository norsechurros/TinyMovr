#!/usr/bin/env python3.10

import rospy
import time
import signal
import sys
import can
import glob
import threading
import tinymovr
import tracemalloc

tracemalloc.start()

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from tinymovr.tee import init_tee
from tinymovr.config import create_device, get_bus_config

AXLE_LENGTH = 0.76  # distance between the left and right wheels
WHEEL_RADIUS = 0.19  # radius of each wheel

class TinyM:
    def find_serial_devices(self, pattern):
        serial_devices = glob.glob(pattern)
        return serial_devices

    def __init__(self):
        rospy.init_node('bldc_node', anonymous=True)

        pattern = '/dev/ttyACM*'
        matching_devices = self.find_serial_devices(pattern)

        if matching_devices:
            channel = matching_devices[0]
        else:
            channel = "/dev/ttyACM0"

        params = get_bus_config()
        params["interface"] = "slcan"
        params["bitrate"] = 1000000
        params["channel"] = channel
        init_tee(can.Bus(**params))

        self.tm1 = create_device(node_id=1)
        self.tm2 = create_device(node_id=2)

        self.tm1.encoder.type = 1
        self.tm1.motor.pole_pairs = 7
        self.tm1.controller.velocity.p_gain = 0.07 ##should be 0.017
        self.tm1.controller.velocity.i_gain = 0.7
        self.tm1.save_config()
        self.tm1.reset()
        time.sleep(1)
        
        self.tm2.encoder.type = 1
        self.tm2.motor.pole_pairs = 7
        self.tm2.controller.velocity.p_gain = 0.06
        self.tm2.controller.velocity.i_gain = 0.7
        self.tm2.save_config()
        self.tm2.reset()
        time.sleep(1)
        
        #self.tm1.controller.calibrate()
        
    def engage(self):
        self.tm1.controller.velocity_mode()
        self.tm1.controller.velocity_setpoint = 0
        time.sleep(1)
        self.tm2.controller.velocity_mode()
        self.tm2.controller.velocity_setpoint = 0
        time.sleep(1)
        
    def cmd_vel_clbk(self, msg):
        try:
            left_w_vel = msg.linear.x - ((msg.angular.z * AXLE_LENGTH) / 2.0)
            right_w_vel = msg.linear.x + ((msg.angular.z * AXLE_LENGTH) / 2.0)

            left_w_rpm = (left_w_vel / (2 * 3.14 * WHEEL_RADIUS)) * 60
            right_w_rpm = -(right_w_vel / (2 * 3.14 * WHEEL_RADIUS)) * 60

            self.tm1.controller.velocity.setpoint = (left_w_rpm / 60) * 29 * 60
            self.tm2.controller.velocity.setpoint = (right_w_rpm / 60) * 29 * 60
            
            print("-----------------------------------------------------")
            print("tm1: " + str(self.tm1.controller))
            
            print("---------------------------MOTOR--------------------------")
            print("tm1: " + str(self.tm1.motor))
            
            print("-----------------------------------------------------")
            print("tm2: " + str(self.tm2.controller))
            
            print("---------------------------MOTOR--------------------------")
            print("tm2: " + str(self.tm2.motor))
            
            
        except tinymovr.channel.ResponseError as e:
            print(f"Error communicating with TinyM: {e}")

    def main(self):
        rospy.loginfo("Robot Motion Control Node started.")
        self.engage()
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_clbk, queue_size=1)
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = TinyM()
        controller.main()
    except rospy.ROSInterruptException:
        pass
