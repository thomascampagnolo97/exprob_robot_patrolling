#! /usr/bin/env python

"""
.. module:: robot_motion
   :platform: Unix
   :synopsis: Script to get the rooms coordinates
.. moduleauthor:: Thomas Campagnolo <s5343274@studenti.unige.it>

ROS Service to move the robot to a desired position specified in the request.

Service:
    - name: /get_coordinate 
    - request containing the desired x and y position

Service response:
    - indicating whether the robot reached the target position or not

"""

import rospy
from exprob_robot_patrolling.srv import GetCoordinates
import actionlib
from move_base_msgs.msg import *
from actionlib_msgs.msg import *

# Import constant name defined to structure the architecture
from exprob_robot_patrolling import architecture_name_mapper as anm

def callback_motion(request):
    """
    
    Callback function for the `/get_coordinate` service. Recives as input the desired coordinates and uses the Action client for `move_base`.
    
    """
    x = request.x
    y = request.y
    print('going to: ', x, y)
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction) #start movebaseaction
    print('subscribed')
    client.wait_for_server() #waiting for response
    
    #create the goal
    goal = MoveBaseGoal()
    #set the goal parameter
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.pose.orientation.w = 1
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    #send the goal
    client.send_goal(goal)
    #wait for result
    wait = client.wait_for_result(timeout=rospy.Duration(50.0))
    if not wait:
        #target not reached, calling cancle goal and return
        print("Mission failed, robot didn't reach goal")
        client.cancel_goal()
        return -1
    return 1 #if reahced the target

def motion_action():
    """
    Initializes the `/get_coordinate` service and the `robot_motion` node.
    """
    rospy.init_node(anm.NODE_ROBOT_MOTION) #setting up the node
    service= rospy.Service('get_coordinate', GetCoordinates, callback_motion)
    print("service ready")
    rospy.spin()

if __name__=="__main__":
    motion_action()