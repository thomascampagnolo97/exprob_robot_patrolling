#!/usr/bin/env python
"""
.. module:: room_scan
   :platform: Unix
   :synopsis: Python code to move the arm such that the scanning is performed 

.. moduleauthor:: Thomas Campagnolo <s5343274@studenti.unige.it>

This script control the joint 1 of the robot arm. 
It does this by publishing commands to the topic "robot_patrolling_explorer/joint1_position_controller/command" and 
subscribing to the topic "robot_patrolling_explorer/joint1_position_controller/state" to monitor the joint's position.

The implemented control of the joint allows the robot arm to perform a rotational movement such as to be able to scan 
the room for patrolling purposes.

Publishes to:
    /robot_patrolling_explorer/joint1_position_controller/command: joint 1 command

Subscribes  to:
    /robot_patrolling_explorer/joint1_position_controller/state: joint 1 control state
    
"""

# Import necessary libraries
import rospy
from control_msgs.msg import JointControllerState
from std_msgs.msg import Float64, Bool

# Import constant name defined to structure the architecture
from exprob_robot_patrolling import architecture_name_mapper as anm



value_joint1 = 0.0
scan_room = 0

def callback_joint1(msg):
    """
    Callback function for the joint position.
    
    Args:
        - JointControllerState 
    """

    global value_joint1
    value_joint1 = msg.process_value


def callback_scan_room(data):
    """ 
    Callback function for the fsm publisher ``/scanning_room``, that modifies the value of the global variable ``scan_room`` and will let the 
    room scan begin.
    
    """

    global scan_room

    if data.data == 0:
        scan_room = 0

    elif data.data == 1:
        scan_room = 1

    return scan_room


def scan_action():
    """"
    This function controls the commands passed to the joint. Joint 1 will rotate to scan the room.
    
    Raises: 
        - ROSInterruptException
    
    """

    global scan_room
    
    rospy.init_node(anm.NODE_ROOM_SCAN, anonymous = True)

    while not rospy.is_shutdown():

        joint1_pose_pub = rospy.Publisher(anm.TOPIC_JOINT1_COMMAND, Float64, queue_size=10)
        rospy.Subscriber(anm.TOPIC_JOINT1_STATE, JointControllerState, callback_joint1)  

        rospy.Subscriber(anm.TOPIC_SCANNING_ROOM, Bool, callback_scan_room)

        if scan_room == 1:
            while value_joint1 > -3.099:
                joint1_pose_pub.publish(-3.10)
            while value_joint1 < 2.399:
                joint1_pose_pub.publish(2.40)
            
            joint1_pose_pub.publish(0.0)
            print(value_joint1)
            rospy.sleep(1.0)
        else:
            rospy.sleep(0.5)



if __name__ == '__main__':
    try:
        scan_action()
    except rospy.ROSInterruptException:
        pass
  