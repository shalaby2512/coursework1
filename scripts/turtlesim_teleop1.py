#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, TeleportAbsolute
import random
import threading


def spawn_turtle():

    rospy.wait_for_service('/turtle1/spawn')  # Wait for the spawn service to be available
    try:
        # Set up the spawn service client
        spawn = rospy.ServiceProxy('/spawn', Spawn)
        # Call the service with x, y position, and theta (orientation) of the turtle
        spawn(5.0, 5.0, 0.0)  # x=5.0, y=5.0, orientation=0.0
        rospy.loginfo("Turtle spawned at (5.0, 5.0) with orientation 0.0")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def initialise():
	rospy.init_node('turtle_teleop', anonymous=False) # init node here
	pub=rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10) # init publisher here
	rate=rospy.Rate(50)
	move_message = Twist()
	spawn_turtle()
	
	while not rospy.is_shutdown():
		key = ord(getch.getch())
		if key==65:
			rospy.loginfo("Up key pressed")
			move_message.linear.x = 1.0
			move_message.angular.z = 0.0
			pub.publish(move_message)
		# move turtle forward here
		elif key==66:
			rospy.loginfo("Down key pressed")
			move_message.linear.x = -1.0
			move_message.angular.z = 0.0
			pub.publish(move_message)
		# move turtle backwards here
		elif key==67:
			move_message.linear.x = 0.0
			move_message.angular.z = -1.0
			pub.publish(move_message)
			rospy.loginfo("left key pressed")
		#move turtle left here

		elif key==68:
			rospy.loginfo("right key pressed")
			move_message.linear.x = 0.0
			move_message.angular.z = 1.0
			pub.publish(move_message)
		#move turtle right here

		rate.sleep()

if __name__ == '__main__':
	initialise()
	
