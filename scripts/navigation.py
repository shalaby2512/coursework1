#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist

from turtlesim.msg import Pose

from math import pow, atan2, sqrt


# Global pose variable

pose = Pose()


def init_node():

    rospy.init_node('turtlebot_target_controller', anonymous=True)

    # Target publisher

    target_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    # Subscriber to collect the current position

    rospy.Subscriber('/turtle1/pose', Pose, update_pose)

    rate = rospy.Rate(10)  # 10Hz

    return target_publisher, rate


def update_pose(data):

    global pose

    # Update the global pose variable with current pose data

    pose = data
    
    # Rounding to 3 decimel places. 

    pose.x = round(pose.x, 3)

    pose.y = round(pose.y, 3)


def direct_distance(target_pose):

    # Calculate the distance between target and current positions using Pythagorean theorem ( a^2=b^2+c^2)

    return sqrt(pow((target_pose.x - pose.x), 2) + pow((target_pose.y - pose.y), 2))


def linear_vel(target_pose, constant=1.5):

    # Calculate the linear speed based on the distance to target.

    if direct_distance(target_pose) < constant:

        return direct_distance(target_pose)

    else:

        return constant * direct_distance(target_pose)


def steering_angle(target_pose):

    # Calculate the angle (theta) to target.

    return atan2(target_pose.y - pose.y, target_pose.x - pose.x)


def angular_vel(target_pose, constant=6):

    # Calculate the angular velocity based on the difference between angles.

    return constant * (steering_angle(target_pose) - pose.theta)


def main():

    try:

        target_publisher, rate = init_node()

        vel_msg = Twist()


        while not rospy.is_shutdown():

            target_pose = Pose()

            

            # Get user input for the goal coordinates ( 0-11 being the boundaries of the turtlesim window)

            target_pose.x = float(input("Enter the x goal (Between 0 to 11): "))

            while target_pose.x < 0 or target_pose.x > 11:

                print("Invalid input. Please re-enter the x-goal (Between 0 and 11).")

                target_pose.x = float(input("Enter the x goal (Between 0 to 11): "))


            target_pose.y = float(input("Enter the y goal (Between 0 to 11): "))

            while target_pose.y < 0 or target_pose.y > 11:

                print("Invalid input. Please re-enter the y-goal (Between 0 and 11).")

                target_pose.y = float(input("Enter the y goal (Between 0 to 11): "))


            distance_tolerance = 0.01  # Tolerance to the target

            

            # Move towards the target while keeping tolerance in mind.

            while direct_distance(target_pose) >= distance_tolerance:


                # Set linear and angular velocities

                vel_msg.linear.x = linear_vel(target_pose)

                vel_msg.angular.z = angular_vel(target_pose)

                # print(f"Remain Distance : {direct_distance(target_pose)} Current X: {vel_msg.linear.x} || Z: {vel_msg.angular.z}")

                print(f"--- Moving forward. Linear X: {round(vel_msg.linear.x, 7)} || Angular Z: {round(vel_msg.angular.z, 7)}")


                # Publish velocity message

                target_publisher.publish(vel_msg)


                # Sleep to maintain the loop rate

                rate.sleep()


            print("")

            print(">>>>> Arrived at Target.")

            print("")

            print("--------------------------------------------")


            # Stop the robot once target is reached

            vel_msg.linear.x = 0

            vel_msg.angular.z = 0

            target_publisher.publish(vel_msg)

            

            break  # The program will stop after arriving at the set target.


    except Exception as e:

        rospy.logerr(f"Error: {e}")

    

    finally:

        # Stopping the robot when done.

        vel_msg.linear.x = 0

        vel_msg.angular.z = 0

        target_publisher.publish(vel_msg)


if __name__ == '__main__':

    try:

        main()

    except rospy.ROSInterruptException:

        pass
	

