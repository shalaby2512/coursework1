#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
from turtlesim.msg import Pose
import math
import random  # Make sure random module is imported

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


# Function to check if two turtles are too close to each other
def is_near_other_turtles(turtle_name, x, y, threshold=1.5):
    turtles = ["turtle1", "turtle2", "turtle3", "turtle4", "turtle5"]
    turtles.remove(turtle_name)  # Remove self from the list

    for other_turtle in turtles:
        other_x, other_y, _ = get_turtle_pose(other_turtle)
        if calculate_distance(x, y, other_x, other_y) < threshold:
            return True
    return False


# Function to move the turtles autonomously with Roomba-like behavior
def autonomous_movement():
    rospy.init_node('turtle_teleop', anonymous=False)

    # Publishers for each turtle's velocity
    turtle_pubs = {
        "turtle1": rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10),
        "turtle2": rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10),
        "turtle3": rospy.Publisher('/turtle3/cmd_vel', Twist, queue_size=10),
        "turtle4": rospy.Publisher('/turtle4/cmd_vel', Twist, queue_size=10),
        "turtle5": rospy.Publisher('/turtle5/cmd_vel', Twist, queue_size=10),
    }

    rate = rospy.Rate(10)  # 10 Hz update rate

    # Roomba-like behavior for all turtles
    while not rospy.is_shutdown():
        for turtle_name in turtle_pubs.keys():
            x, y, theta = get_turtle_pose(turtle_name)
            move_message = Twist()

            # Check if the turtle is near a wall (boundary)
            if is_near_wall(x, y, threshold=1.5):
                rospy.loginfo(f"Turtle {turtle_name} is near a wall. Turning!")

                # Handle rotation based on the wall hit
                move_message = handle_wall_rotation(x, y)

                # Rotate in place while avoiding a wall, then move forward
                move_message.linear.x = 1.0  # Increased forward speed
                move_message.angular.z = move_message.angular.z  # Continue rotating
                turtle_pubs[turtle_name].publish(move_message)

            # Check if the turtle is too close to another turtle
            elif is_near_other_turtles(turtle_name, x, y, threshold=1.0):
                rospy.loginfo(f"Turtle {turtle_name} is too close to another turtle. Turning!")

                # Rotate to avoid collision with other turtles
                move_message.angular.z = random.choice([math.pi / 2, -math.pi / 2])  # Random rotation

                # Move forward with increased speed
                move_message.linear.x = 1.0  # Increased forward speed
                turtle_pubs[turtle_name].publish(move_message)

            else:
                # Move forward if no wall or collision detected
                rospy.loginfo(f"Turtle {turtle_name} moving forward")
                move_message.linear.x = 1.0  # Increased forward speed
                move_message.angular.z = 0.0  # No rotation
                turtle_pubs[turtle_name].publish(move_message)

        rate.sleep()


# Function to get the pose of the turtle

def get_turtle_pose(turtle_name):
    pose = rospy.wait_for_message(f"/{turtle_name}/pose", Pose, timeout=1)
    return (pose.x, pose.y, pose.theta)


# Main function to initialize the program
def main():
    rospy.init_node('turtle_teleop', anonymous=False)

    # Spawn multiple turtles at starting positions with opposite directions from each other
    
    spawn_turtle("turtle1", 5.0, 5.0, 0.0)     # Facing right (0 radians)
    spawn_turtle("turtle2", 3.0, 3.0, math.pi) # Facing left (pi radians)
    spawn_turtle("turtle3", 7.0, 3.0, math.pi / 2)  # Facing up (pi/2 radians)
    spawn_turtle("turtle4", 3.0, 7.0, -math.pi / 2) # Facing down (-pi/2 radians)
    spawn_turtle("turtle5", 7.0, 7.0, math.pi / 4)  # Facing diagonally (pi/4 radians)

    # Run the autonomous movement function
    autonomous_movement()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

