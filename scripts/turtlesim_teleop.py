#!/usr/bin/env python3
import rospy
import subprocess
import threading
import getch
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn

# Function to spawn a turtle
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

# Function to launch the turtlesim node


# Function to control the turtle via getch input
def initialise():
    rospy.init_node('turtle_teleop', anonymous=False)  # Init node here
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)  # Init publisher here
    rate = rospy.Rate(50)
    move_message = Twist()

    while not rospy.is_shutdown():
        key = ord(getch.getch())  # Wait for key press synchronously
        if key == 65:  # Up arrow key
            rospy.loginfo("Up key pressed")
            move_message.linear.x = 1.0
            move_message.angular.z = 0.0
            pub.publish(move_message)
        elif key == 66:  # Down arrow key
            rospy.loginfo("Down key pressed")
            move_message.linear.x = -1.0
            move_message.angular.z = 0.0
            pub.publish(move_message)
        elif key == 67:  # Right arrow key
            rospy.loginfo("Right key pressed")
            move_message.linear.x = 0.0
            move_message.angular.z = -1.0  # Rotate left (clockwise)
            pub.publish(move_message)
        elif key == 68:  # Left arrow key
            rospy.loginfo("Left key pressed")
            move_message.linear.x = 0.0
            move_message.angular.z = 1.0  # Rotate right (counterclockwise)
            pub.publish(move_message)

        rate.sleep()

# Main function that runs everything
def main():
    # Start the ROS node
    rospy.init_node('turtle_teleop', anonymous=False)

    # Run launch_turtlesim() and spawn_turtle() in separate threads
    #launch_thread = threading.Thread(target=launch_turtlesim)
    spawn_thread = threading.Thread(target=spawn_turtle)

    # Start the threads
    #launch_thread.start()
    spawn_thread.start()

    # Run the turtle control in the main thread
    initialise()

    # Wait for threads to finish (this step ensures that the subprocesses finish)
    #launch_thread.join()
    spawn_thread.join()




if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

