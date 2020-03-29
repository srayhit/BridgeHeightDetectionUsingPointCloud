#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math
import sys

height = 0

def linear_move(speed, distance, direction, arg_height):
    
	global height
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

	      
	t0 = rospy.Time.now().to_sec()
	rate=rospy.Rate(100)
	current_distance = 0

	while(current_distance < distance):
		#print height
		if height < arg_height and height!=0 and height!=-1:
			publisher.publish(Twist())#Stop all movement if the bridge is detected to be too low
			break
		else:
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

if __name__ == '__main__':
    try:
	myargv = rospy.myargv(argv=sys.argv)
	#print myargv
	rospy.init_node('navigation_node', anonymous=True)
	publisher = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
	subscriber = rospy.Subscriber('/bridgeheight', Float64, getHeight)
    	message = Twist()
	linear_move(0.1,5,1, float(myargv[1]))#Need to convert to float
	rospy.sleep(1)

    except rospy.ROSInterruptException:
        pass
