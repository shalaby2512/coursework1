#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
from turtlesim.msg import Pose
import math


# Function to spawn a turtle at a specific position with a unique name
def spawn_turtle(turtle_name, x, y, theta):
    rospy.wait_for_service('/spawn')  # Wait for the spawn service to be available
    try:
        spawn = rospy.ServiceProxy('/spawn', Spawn)
        spawn(x, y, theta, turtle_name)
    except rospy.ServiceException as e:
        rospy.logerr(f'Service call failed: {e}')


# Function to calculate distance between two points (x1, y1) and (x2, y2)
def calculate_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


# Function to check if the turtle is near the wall
def is_near_wall(x, y, threshold=1.0):
    if x <= threshold or x >= 11.0 - threshold or y <= threshold or y >= 11.0 - threshold:
        return True
    return False


# Function to handle rotation based on wall proximity
def handle_wall_rotation(x, y):
    move_message = Twist()

    # Rotate based on which wall is being hit
    if x <= 1.0:  # Left wall
        move_message.angular.z = math.pi / 2  # Rotate right
    elif x >= 10.0:  # Right wall
        move_message.angular.z = -math.pi / 2  # Rotate left
    elif y <= 1.0:  # Bottom wall
        move_message.angular.z = -math.pi / 2  # Rotate left
    elif y >= 10.0:  # Top wall
        move_message.angular.z = math.pi / 2  # Rotate right

    return move_message


# Function to move the turtle autonomously with Roomba-like behavior
def autonomous_movement():
    rospy.init_node('turtle_teleop', anonymous=False)

    # Publisher for the turtle's velocity
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz update rate

    # Roomba-like behavior
    while not rospy.is_shutdown():
        x, y, theta = get_turtle_pose('turtle1')
        move_message = Twist()

        # Check if the turtle is near a wall (boundary)
        if is_near_wall(x, y, threshold=1.0):
            rospy.loginfo("Turtle is near a wall. Turning!")

            # Handle rotation based on the wall hit
            move_message = handle_wall_rotation(x, y)

            # Only rotate if the turtle is not already rotating
            pub.publish(move_message)
            rospy.sleep(1)  # Rotate for 1 second

            # Move faster forward after turning
            move_message.linear.x = 1.0  # Increased forward speed
            move_message.angular.z = 0.0  # No rotation
            pub.publish(move_message)
            rospy.sleep(2)  # Move for 2 seconds after rotation

        else:
            # Move forward if no wall detected
            rospy.loginfo("Turtle moving forward")
            move_message.linear.x = 1.0  # Increased forward speed
            move_message.angular.z = 0.0  # No rotation
            pub.publish(move_message)

        rate.sleep()


# Function to get the pose of the turtle
def get_turtle_pose(turtle_name):
    pose = rospy.wait_for_message(f"/{turtle_name}/pose", Pose, timeout=1)
    return (pose.x, pose.y, pose.theta)


# Main function to initialize the program
def main():
    rospy.init_node('turtle_teleop', anonymous=False)

    # Launch the turtlesim node
    # Uncomment the lines below if you want to programmatically launch turtlesim
    # rospy.loginfo("Launching turtlesim_node...")
    # subprocess.Popen(["roslaunch", "turtlesim", "turtlesim_node.launch"])
    # rospy.sleep(2)  # Give it time to initialize the turtlesim environment

    # Spawn a turtle at a starting position
    spawn_turtle("turtle1", 5.0, 5.0, 0.0)  # Start at (5.0, 5.0) with 0.0 orientation

    # Run the autonomous movement function
    autonomous_movement()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

