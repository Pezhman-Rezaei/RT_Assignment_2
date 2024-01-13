#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry
import actionlib
import assignment_2_2023.msg
from assignment_2_2023.msg import Vel
from assignment_2_2023.msg import PlanningAction, PlanningGoal, PlanningResult
from std_srvs.srv import SetBool
from actionlib_msgs.msg import GoalStatus

def initialize_ros_node():
    rospy.init_node('set_target_client')
    rospy.loginfo("ROS Node 'set_target_client' initialized")

def initialize_publisher():
    return rospy.Publisher("/pos_vel", Vel, queue_size=1)

def initialize_action_client():
    client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2023.msg.PlanningAction)
    client.wait_for_server()
    return client

def get_user_command():
    return input("Press 'I' to set a new goal or 'O' to cancel the current goal: ").upper()

def get_current_target_position():
    return rospy.get_param('/des_pos_x'), rospy.get_param('/des_pos_y')

def create_goal(x, y):
    goal = assignment_2_2023.msg.PlanningGoal()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    return goal

def set_new_goal(client, goal):
    client.send_goal(goal)

def cancel_current_goal(client):
    client.cancel_goal()

def handle_user_input():
    while not rospy.is_shutdown():
        rospy.Subscriber("/odom", Odometry, publish_position_velocity)
        command = get_user_command()

        if command == 'I':
            try:
                input_x = float(input("Enter the x-coordinate for the new goal: "))
                input_y = float(input("Enter the y-coordinate for the new goal: "))
            except ValueError:
                rospy.logwarn("Invalid input. Please enter a valid number.")
                continue

            rospy.set_param('/des_pos_x', input_x)
            rospy.set_param('/des_pos_y', input_y)
            new_goal = create_goal(input_x, input_y)
            set_new_goal(action_client, new_goal)

        elif command == 'O':
            cancel_current_goal(action_client)
            rospy.loginfo("Current goal has been cancelled")

        else:
            rospy.logwarn("Invalid command. Please enter 'I' or 'O'.")

def publish_position_velocity(msg):
    current_pos = msg.pose.pose.position
    current_vel_linear = msg.twist.twist.linear
    current_vel_angular = msg.twist.twist.angular

    pos_and_vel = Vel()
    pos_and_vel.pos_x = current_pos.x
    pos_and_vel.pos_y = current_pos.y
    pos_and_vel.vel_x = current_vel_linear.x
    pos_and_vel.vel_z = current_vel_angular.z

    publisher.publish(pos_and_vel)

def main():
    initialize_ros_node()
    global publisher
    publisher = initialize_publisher()
    global action_client
    action_client = initialize_action_client()
    handle_user_input()

if __name__ == '__main__':
    main()

