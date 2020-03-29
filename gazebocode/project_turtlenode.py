#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
#from geometry_msgs.msg import ModelStates
#inlcude "gazebo_msgs/ModelStates.h"
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float64
import math
import sys

height = 0
thetaData = 0

def linear_move(speed, distance, direction, arg_height):
    
	global height
	global thetaData
	print arg_height
	if(direction):
   	   	message.linear.x = abs(speed)
		print "Moving ",distance, " distance forward"
	else:
   	   	message.linear.x = -abs(speed)
		print "Moving ",distance, " distance backward"
	
   	message.linear.y = 0
   	message.linear.z = 0
   	message.angular.x = 0
   	message.angular.y = 0
   	message.angular.z = 0
   	PI = 3.1415926535897
   	currentAngle = thetaData*2*PI/360
   	angularSpeed = 0.7*2*PI/360
       
   	t0 = rospy.Time.now().to_sec()
   	rate=rospy.Rate(1)
   	current_distance = 0

   	while(current_distance < distance):
		#print height
		while(thetaData > 0.1):
			publisher.publish(Twist())
			print "inside1st"
			t1=rospy.Time.now().to_sec()
			message.linear.x = 0.2
			message.angular.z = -0.2
			publisher.publish(message)
		while(thetaData < -0.1):
			print "inside2nd",thetaData
			publisher.publish(Twist())
			t1=rospy.Time.now().to_sec()
			message.linear.x = 0.2
			message.angular.z = 0.2
			publisher.publish(message)

		if height < arg_height and height!=-1:
			print "Not moving",height
			publisher.publish(Twist())#Stop all movement if the bridge is detected to be too high
			continue
		else:
			print "Moving"
			message.linear.x = abs(speed)
    		publisher.publish(message)
    		t1=rospy.Time.now().to_sec()
    		current_distance= speed*(t1-t0)
	rate.sleep()
	
	#Force stop with empty Twist message
	publisher.publish(Twist())
	rospy.sleep(1)

def getHeight(data):
	global height
	height = data.data

def getModelState(data):
	global thetaData
	rot_q = data.pose[1].orientation
	(roll,pitch,theta) = euler_from_quaternion ([rot_q.x,rot_q.y,rot_q.z,rot_q.w])
	thetaData = theta
	#print thetaData

if __name__ == '__main__':
    try:
	myargv = rospy.myargv(argv=sys.argv)
	#print myargv
	rospy.init_node('my_initials', anonymous=True)
	publisher = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
	subscriber2 = rospy.Subscriber('/gazebo/link_states',LinkStates, getModelState)
	subscriber = rospy.Subscriber('/bridgeheight', Float64, getHeight)
	message = Twist()
	linear_move(1,100,1, float(myargv[1]))#Need to convert to float
	rospy.sleep(1)

    except rospy.ROSInterruptException:
        pass
