#!/usr/bin/env python3
import rospy
import subprocess
import getch
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
import time

# Function to spawn a turtle at specific position with unique name
def spawn_turtle(turtle_name, x, y, theta):
    rospy.wait_for_service('/spawn')  # Wait for the spawn service to be available
    try:
        # Set up the spawn service client
        spawn = rospy.ServiceProxy('/spawn', Spawn)
        # Call the service with x, y position, theta (orientation), and the turtle name
        spawn(x, y, theta, turtle_name)  # Include the name parameter for the turtle
        rospy.loginfo(f"Turtle {turtle_name} spawned at ({x}, {y}) with orientation {theta}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

# Function to launch the turtlesim node
def launch_turtlesim():
    """
    Launch the turtlesim_node programmatically using roslaunch
    """
    try:
        # Launch the turtlesim node using roslaunch
        rospy.loginfo("Launching turtlesim_node...")
        subprocess.Popen(["roslaunch", "turtlesim", "turtlesim.launch"])  # Use turtlesim.launch instead of turtlesim_node.launch
        rospy.sleep(2)  # Give it some time to initialize before spawning turtles
    except Exception as e:
        rospy.logerr(f"Error launching turtlesim_node: {e}")

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

    # Launch the turtlesim node
    launch_turtlesim()

    # Wait before spawning turtles (adjust time if necessary)
    rospy.sleep(5)  # Increase the sleep time to give more time for turtlesim to initialize

    # Spawn multiple turtles at different positions
    spawn_turtle("turtle1", 5.0, 5.0, 0.0)
    spawn_turtle("turtle2", 2.0, 2.0, 0.0)
    spawn_turtle("turtle3", 8.0, 2.0, 0.0)
    spawn_turtle("turtle4", 2.0, 8.0, 0.0)
    spawn_turtle("turtle5", 8.0, 8.0, 0.0)

    # Run the turtle control in the main thread
    initialise()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

