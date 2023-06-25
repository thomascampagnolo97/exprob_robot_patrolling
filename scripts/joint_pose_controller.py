#!/usr/bin/env python
"""
.. module:: joint_pose_controller
   :platform: Unix
   :synopsis: Python code to control the joints of the arm
.. moduleauthor:: Thomas Campagnolo <s5343274@studenti.unige.it>

The purpose of this script is to control the joints of the robot arm to the desired position. 
To do that, it publishes commands to the topic "robot_patrolling_explorer/joint*_position_controller/command" and 
subscribes to the topic "robot_patrolling_explorer/joint*_position_controller/state" to monitor the joint's position.

The implemented control of the joints allows the robot arm to perform a movement such as to be able to detect and scan 
all the markers in order to build the ontology and the map of the environment.

Publishes to:
    /robot_patrolling_explorer/joint1_position_controller/command: joint 1 command
    /robot_patrolling_explorer/joint2_position_controller/command: joint 2 command
    /robot_patrolling_explorer/joint3_position_controller/command: joint 3 command

Subscribes  to:
    /robot_patrolling_explorer/joint1_position_controller/state: joint 1 control state
    /robot_patrolling_explorer/joint2_position_controller/state: joint 2 control state
    /robot_patrolling_explorer/joint3_position_controller/state: joint 3 control state
"""

# Import necessary libraries
import rospy
from control_msgs.msg import JointControllerState
from std_msgs.msg import Float64

# Import constant name defined to structure the architecture
from exprob_robot_patrolling import architecture_name_mapper as anm


value_joint1 = 0.0
value_joint2 = 0.0

def callback_joint1(msg):
    """
    Callback function for the joint1 position.
    
    Args:
        - JointControllerState 
    """
    global value_joint1
    value_joint1 = msg.process_value

def callback_joint2(msg):
    """
    Callback function for the joint2 position.
    
    Args:
        - JointControllerState 
    """
    global value_joint2
    value_joint2 = msg.process_value


def callback_joint3(msg):
    """
    Callback function for the joint3 position.
    
    Args:
        - JointControllerState 
    """
    global value_joint3
    value_joint3 = msg.process_value


    return value_joint3


def joint_action():
    # Create ROS node
    """"
    This function controls the commands passed to the joints. Joint 1,2 and 3 will rotate to scan the room."
    
    Raises: 
        - ROSInterruptException
    
    """
    
    rospy.init_node(anm.NODE_JOINTS_CONTROL, anonymous = True)

    joint1_pose_pub = rospy.Publisher(anm.TOPIC_JOINT1_COMMAND, Float64, queue_size=10)
    rospy.Subscriber(anm.TOPIC_JOINT1_STATE, JointControllerState, callback_joint1)

    joint2_pose_pub = rospy.Publisher(anm.TOPIC_JOINT2_COMMAND, Float64, queue_size=10)
    rospy.Subscriber(anm.TOPIC_JOINT2_STATE, JointControllerState, callback_joint2)

    joint3_pose_pub = rospy.Publisher(anm.TOPIC_JOINT3_COMMAND, Float64, queue_size=100)
    rospy.Subscriber(anm.TOPIC_JOINT3_STATE, JointControllerState, callback_joint3)    

    while value_joint1 > -3.099:
        print("Value of joint 1: ", value_joint1)
        joint1_pose_pub.publish(-3.14)
        joint2_pose_pub.publish(0.60)
        joint3_pose_pub.publish(-0.25)
        
    while value_joint1 < 2.399:
        print("Value of joint 1: ", value_joint1)
        joint1_pose_pub.publish(2.40)
        joint2_pose_pub.publish(0.60)
        joint3_pose_pub.publish(-0.25)
    

    while value_joint2 > -0.55:
        print("Value of joint 2: ", value_joint2)
        joint2_pose_pub.publish(-0.57)
        joint3_pose_pub.publish(0.33)
        
    if value_joint2 <= -0.55:
        while value_joint1 > -1.59:
            print("Value of joint 1: ", value_joint1)
            joint1_pose_pub.publish(-1.60)
            joint3_pose_pub.publish(0.33)

    joint1_pose_pub.publish(0.0)
    joint2_pose_pub.publish(0.0)
    joint3_pose_pub.publish(-0.25)
        
    rospy.sleep(0.5)


if __name__ == '__main__':
    try:
        joint_action()
    except rospy.ROSInterruptException:
        pass
  