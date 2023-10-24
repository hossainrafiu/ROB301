#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

import matplotlib.pyplot as plt
import math
import numpy as np


class KalmanFilter(object):
    def __init__(self, h, d, x_0, Q, R, P_0):
        self.h = h
        self.d = d

        self.Q = Q
        self.R = R
        self.P = P_0
        self.x = x_0

        self.u = 0  # initialize the cmd_vel input
        self.phi = 0  # initialize the measurement input

        self.state_pub = rospy.Publisher("state", Float64, queue_size=1)
        self.scan_sub = rospy.Subscriber(
            "scan_angle", Float64, self.scan_callback, queue_size=1
        )
        self.cmd_sub = rospy.Subscriber("cmd_vel_noisy", Twist, self.cmd_callback)

    def cmd_callback(self, cmd_msg):
        self.u = cmd_msg.linear.x

    ## updates self.phi with the most recent measurement of the tower.
    def scan_callback(self, msg):
        self.phi = msg.data

    ## call within run_kf to update the state with the measurement
    def predict(self, u=0):
        """
        TODO: update state via the motion model, and update the covariance with the process noise
        """
        self.xk1k = self.x + u*1/30
        self.P = self.P + self.Q
        return self.xk1k

    ## call within run_kf to update the state with the measurement
    def measurement_update(self):
        """
        TODO: update state when a new measurement has arrived using this function
        """
        return

    def run_kf(self):
        current_input = self.u
        current_measurement = self.phi

        """
        TODO: complete this function to update the state with current_input and current_measurement
        """
        self.x = self.predict(current_input)
        self.state_pub.publish(self.x)


if __name__ == "__main__":
    rospy.init_node("lab4")

    h = 0.60  # y distance to tower
    d = 0.60 * 3  # x distance to tower (from origin)

    x_0 = 0  # initial state position

    Q = 1  # TODO: Set process noise covariance
    R = 1  # TODO: measurement noise covariance
    P_0 = 1  # TODO: Set initial state covariance

    kf = KalmanFilter(h, d, x_0, Q, R, P_0)
    rospy.sleep(1)

    rate = rospy.Rate(30)
    deliveries = [61, 122, 244, 305]
    current_delivery = deliveries.pop()
    cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    time = rospy.get_time()
    while not rospy.is_shutdown():
        kf.run_kf()
        if (current_delivery - kf.x) < 10:
            u = 0.1
            time = rospy.get_time()
        elif rospy.get_time() - time < 2:
            u = 0
        else: 
            current_delivery = deliveries.pop()
        cmd_pub.publish(u)
        rate.sleep()
