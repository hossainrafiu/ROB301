#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String

def get_yaw_from_quaternion(q):
	siny_cosp = 2* (q.w*q.z + q.x*q.y)
	cosy_cosp = 1 - 2*(q.y*q.y + q.z*q.z)
	yaw = math.atan2(siny_cosp,cosy_cosp)
	return yaw

def callback(odom_data):
	global cur_pose
	point=odom_data.pose.pose.position
	quart=odom_data.pose.pose.orientation
	theta=get_yaw_from_quaternion(quart)
	cur_pose = (point.x, point.y, theta)
	rospy.loginfo(cur_pose)


def main():
	rospy.init_node("lab02")    
	cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
	rate = rospy.Rate(10)
	twist=Twist()
	
	'''
	initial_time = rospy.get_time()
	distance = 0
	while not rospy.is_shutdown():
		time = rospy.get_time() - initial_time
		twist.linear.x = 0.1
		twist.angular.z = 0.5 * math.sin(math.pi/0.25*time)
		cmd_pub.publish(twist)
		rate.sleep()
	twist.linear.x = 0
	twist.angular.z = 0
	'''
	
	
	angle = 0
	x = 0
	while not rospy.is_shutdown(): # set to run for 2m
		twist.linear.x = 0.1 
		w=0.1 / (1+(2*math.pi*math.cos(math.pi/0.25*x))**2)**(3/2)/(-2/0.25*math.pi**2*math.sin(math.pi/0.25*x)+0.001) + 0.001
		twist.angular.z = w
		cmd_pub.publish(twist)
		x+= 0.1*math.sin(angle)/10
		angle += w
		rate.sleep()
	twist.linear.x = 0
	twist.angular.z = 0
	
	
	'''
	distance = 0
	while distance < 1.76591:
		twist.linear.x = 0.2
		twist.angular.z = 0.0
		cmd_pub.publish(twist)
		distance += 0.2/10 #divide by rate
		rate.sleep()
	twist.linear.x = 0
	
	initial_angle = 0
	while initial_angle < 2.06155:
		twist.linear.x = 0.2
		twist.angular.z = 0.8
		cmd_pub.publish(twist)
		initial_angle += 0.5/10 #divide by rate
		rate.sleep()
	twist.linear.x = 0
	twist.angular.z = 0
	'''

if __name__ == "__main__":
    main()
