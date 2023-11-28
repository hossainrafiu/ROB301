#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, UInt32

import matplotlib.pyplot as plt
import math
import numpy as np

actual_index = 320
desired_index = 0

class Controller(object):
    def __init__(self):
        # publish motor commands
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        # subscribe to detected line index
        self.color_sub = rospy.Subscriber(
            "line_idx", UInt32, self.camera_callback, queue_size=1
        )

    def camera_callback(self, msg):
        """Callback for line index."""
        global desired_index
        desired_index = msg.data
        #rospy.loginfo(msg.data)

    def follow_the_line(self, max_speed):
        integral = 0
        derivative = 0
        lasterror = 0
        kp = 0.002
        ki = 0.00002
        kd = 0.002
        fix_heading = False
        twist = Twist()
        #rate = rospy.Rate(30)
    
        error = desired_index - actual_index
        if abs(integral) > 10000:
            integral = 0
        integral = integral + error
        derivative = error - lasterror
        twist.angular.z = (- kp * error) + (- ki * integral) + (-kd * derivative)
        
        if abs(error) > 280:
            fix_heading = True
        if not fix_heading:
            twist.linear.x = max_speed/(1+math.exp(abs(error+derivative)/50-2))
        else:
            twist.linear.x = 0

            if abs(error) < 80:
                fix_heading = False
        
        #rospy.loginfo(str(error) + ", " + str(integral) + ", " + str(derivative) + ", " + str(twist.angular.z) + ", " + str(twist.linear.x))
        
        self.cmd_pub.publish(twist)
        lasterror = error
        #rate.sleep()

class KalmanFilter(object):
    def __init__(self, h, d, x_0, Q, R, P_0):
        self.h = h
        self.d = d

        self.Q = Q
        self.R = R
        self.P = P_0
        self.x = x_0

        self.angle_sum = 0
        self.angle_count = 0
        self.D = 0
        self.s = 0
        self.S = 0
        self.W = 0
        self.phi_est = 0

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
        if msg.data > 0:
            expected_range = 0.7 # rad
            print(self.phi_est, msg.data)
            if abs(msg.data - self.phi_est) < expected_range:
                self.phi = msg.data
                self.measurement_update()
        
        """
        if self.phi > 0:
            self.angle_sum += self.phi - 1.57
            self.angle_count += 1
            rospy.loginfo(str(self.angle_sum/self.angle_count))
        """

    ## call within run_kf to update the state with the measurement
    def predict(self, u=0):
        """
        TODO: update state via the motion model, and update the covariance with the process noise
        """
        self.x = self.x + u*1/30
        self.phi_est = math.atan(self.h/(self.d-self.x))
        if self.d-self.x < 0:
            self.phi_est += math.pi
        self.P = self.P + self.Q
        return

    ## call within run_kf to update the state with the measurement
    def measurement_update(self):
        """
        TODO: update state when a new measurement has arrived using this function
        """
        self.D = self.h/(self.h**2 + (self.d-self.x)**2)
        self.s = self.phi - self.phi_est
        
        self.S = self.D*self.P*self.D + self.R
        self.W = self.P*self.D + self.R
        self.P = self.P - self.W*self.S*self.W
        
        self.x = self.x + self.W*self.s
        return
    
    def run_kf(self):
        current_input = self.u
        """
        TODO: complete this function to update the state with current_input and current_measurement
        """
        self.predict(current_input)
        
        #print(self.x, self.phi, self.phi_est, self.s, self.S, self.W)
        self.state_pub.publish(self.x)


if __name__ == "__main__":
    rospy.init_node("lab4")

    h = 0.60  # y distance to tower
    d = 0.60 * 3  # x distance to tower (from origin)

    x_0 = 0  # initial state position

    Q = 0.005**2  # TODO: Set process noise covariance
    R = 0.05  # TODO: measurement noise covariance
    P_0 = Q  # TODO: Set initial state covariance

    kf = KalmanFilter(h, d, x_0, Q, R, P_0)
    rospy.sleep(1)
    controller = Controller()
    rate = rospy.Rate(30)
    deliveries = [3.05, 2.44, 1.22, .61]
    current_delivery = deliveries.pop()
    rospy.loginfo("Current Delivery =" + str(current_delivery))
    time = rospy.get_time()
    stop = False
    P_arr = []
    while not rospy.is_shutdown():
        kf.run_kf()
        #print(str(kf.x) + " " + str(current_delivery))
        if (current_delivery - kf.x) > .02 and not stop:
            controller.follow_the_line(0.15)
            time = rospy.get_time()
        elif rospy.get_time() - time < 2:
            stop = True
            controller.follow_the_line(0)
            pass
        else: 
            if len(deliveries) == 0:
                break
            current_delivery = deliveries.pop()
            stop = False
            rospy.loginfo("Current Delivery =" + str(current_delivery))
        P_arr.append(kf.P)
        rate.sleep()
    while not rospy.is_shutdown():
        kf.state_pub.publish(P_arr.pop(0))
        rate.sleep()
    