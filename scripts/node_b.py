#!/usr/bin/env python3

import rospy
from assignment_2_2023.srv import Input, InputResponse

def initialize_ros_node():
    rospy.init_node('last_target_service')
    rospy.loginfo("ROS Node 'last_target_service' initialized")

def provide_input_service():
    rospy.Service('input', Input, result_callback)

def result_callback(_):
    response = InputResponse()
    last_des_x = rospy.get_param('/des_pos_x')
    last_des_y = rospy.get_param('/des_pos_y')
    response.input_x = last_des_x
    response.input_y = last_des_y
    return response

def spin_node():
    rospy.spin()

def main():
    initialize_ros_node()
    provide_input_service()
    spin_node()

if __name__ == "__main__":
    main()

