#!/usr/bin/env python3

import rospy
import math
from assignment_2_2023.msg import Vel
from assignment_2_2023.srv import Ave_pos_vel, Ave_pos_velResponse

# Define global variables for distance, average velocity, x, and y positions
distance = 0
average_vel_x = 0
current_x = 0
current_y = 0

def initialize_ros_node():
    rospy.init_node('info_service')
    rospy.loginfo("ROS Node 'info_service' initialized")

def provide_info_service():
    rospy.Service("info_service", Ave_pos_vel, get_values)
    rospy.Subscriber("/pos_vel", Vel, get_distance_and_average_velocity)

def get_distance_and_average_velocity(msg):
    global distance, average_vel_x, current_x, current_y

    des_x, des_y = rospy.get_param('/des_pos_x'), rospy.get_param('/des_pos_y')
    velocity_window_size = rospy.get_param('/window_size')

    actual_x, actual_y = msg.pos_x, msg.pos_y
    des_coordinates = [des_x, des_y]
    actual_coordinates = [actual_x, actual_y]

    distance = math.dist(des_coordinates, actual_coordinates)

    if isinstance(msg.vel_x, list):
        vel_data = msg.vel_x[-velocity_window_size:]
    else:
        vel_data = [msg.vel_x]

    # Calculate the average velocity
    total_vel_x = sum(vel_data)
    num_samples = min(len(vel_data), velocity_window_size)
    average_vel_x = total_vel_x / num_samples if num_samples > 0 else 0

    # Update current x and y positions
    current_x, current_y = actual_x, actual_y

def get_values(_):
    return Ave_pos_velResponse(distance, average_vel_x)

def timer_callback(event):
    # This callback is called every 0.5 seconds (as per the timer)
    rospy.loginfo(f"Current X: {current_x}, Current Y: {current_y}, Distance: {distance}, Average Velocity: {average_vel_x}")

def spin_node():
    # Create a timer that calls the timer_callback every 0.5 seconds
    rospy.Timer(rospy.Duration(0.5), timer_callback)
    rospy.spin()

def main():
    initialize_ros_node()
    provide_info_service()
    spin_node()

if __name__ == "__main__":
    main()

