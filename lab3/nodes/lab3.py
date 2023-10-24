#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt32
from math import exp

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
        rospy.loginfo(msg.data)

    def follow_the_line(self):
        integral = 0
        derivative = 0
        lasterror = 0
        kp = 0.003
        ki = 0.00003
        kd = 0.001
        twist = Twist()
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            error = desired_index - actual_index
            if abs(integral) > 10000:
                integral = 0
            integral = integral + error
            derivative = error - lasterror
            twist.angular.z = (- kp * error) + (- ki * integral) + (-kd * derivative)
            
            twist.linear.x = 0.2/(1+exp(abs(error+derivative)/50-2)) + 0.1
            
            rospy.loginfo(str(error) + ", " + str(integral) + ", " + str(derivative) + ", " + str(twist.angular.z) + ", " + str(twist.linear.x))
            
            self.cmd_pub.publish(twist)
            lasterror = error
            rate.sleep()

    def follow_the_line3(self):
        integral = 0
        kp = 0.003
        ki = 0.00003
        twist = Twist()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            error = desired_index - actual_index
            twist.linear.x = 0.1
            integral = integral + error
            twist.angular.z = (- kp * error) + (- ki * integral)
            rospy.loginfo(str(error) + ", " + str(integral) + ", " + str(twist.angular.z))
            self.cmd_pub.publish(twist)
            rate.sleep()

    def follow_the_line2(self):
        kp = 0.003
        twist = Twist()
        twist.linear.x = 0.1
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            error = desired_index - actual_index
            rospy.loginfo(error)
            twist.angular.z = - kp * error

            self.cmd_pub.publish(twist)
            rate.sleep()

    def follow_the_line1(self):
        twist = Twist()
        twist.linear.x = 0.1
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            error = desired_index - actual_index
            rospy.loginfo(error)
            if error < 0:
                twist.angular.z = 0.6
            if error > 0:
                twist.angular.z = -0.6
            if error == 0:
                twist.angular.z = 0

            self.cmd_pub.publish(twist)
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("lab3")
    controller = Controller()
    controller.follow_the_line()
