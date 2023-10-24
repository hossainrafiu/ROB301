#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

cur_pose = (0,0,0)

def callback(odom_data):
	global cur_pose
	point=odom_data.pose.pose.position
	quart=odom_data.pose.pose.orientation
	theta=get_yaw_from_quaternion(quart)
	cur_pose = (point.x, point.y, theta)
	rospy.loginfo(cur_pose)

def main():
	rospy.init_node("motor_node")
	cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
	odom_subscriber=rospy.Subscriber('odom',Odometry,callback,queue_size=1)
	rate = rospy.Rate(10)
	initial = (0,0,0)
	rospy.loginfo(initial)
	
	while not rospy.is_shutdown():
	    
		twist=Twist()
		twist.linear.x=1
		##twist.angular.z=3.14
		cmd_pub.publish(twist)    

		rate.sleep()
		
		if initial == (0,0,0):
			initial = cur_pose
			rospy.loginfo(initial)
			
		distance_traveled = math.sqrt((initial[0]-cur_pose[0])**2 + (initial[1]-cur_pose[1])**2 )
		rospy.loginfo(distance_traveled)
		if distance_traveled >=1:
		    	break
	
	
	initial = cur_pose
	previous_angular = 0 	
	while not rospy.is_shutdown():
	    
		twist=Twist()
		
		twist.angular.z=0.5
		cmd_pub.publish(twist)    

		rate.sleep()
		
		angular_distance = cur_pose[2] - initial[2] + 0.01
		if angular_distance<0:
			angular_distance+=6.28
		rospy.loginfo(previous_angular)
		rospy.loginfo(angular_distance)
		if previous_angular-angular_distance > 6: break
		previous_angular=angular_distance
	    	
def get_yaw_from_quaternion(q):
	siny_cosp = 2* (q.w*q.z + q.x*q.y)
	cosy_cosp = 1 - 2*(q.y*q.y + q.z*q.z)
	yaw = math.atan2(siny_cosp,cosy_cosp)
	return yaw

if __name__ == "__main__":
    main()
