#!/usr/bin/env python
"""
.. module:: joint_pose_modifier
   :platform: Unix
   :synopsis: Script to move the arm
.. moduleauthor:: Thomas Campagnolo <s5343274@studenti.unige.it>

The purpose of this script is to control the joint of a robot and move it to a desired position. 
It does this by publishing commands to the topic "myRob/joint1_position_controller/command" and 
subscribing to the topic "myRob/joint1_position_controller/state" to monitor the joint's position. 
This allows the script to issue successive commands based on the current position of the robot's joint.

"""

# Import necessary libraries
import rospy
from control_msgs.msg import JointControllerState
from std_msgs.msg import Float64


value_joint1 = 0.0
value_joint2 = 0.0

def callback_joint1(msg):
    """
    Callback function for the joint position.
    
    Args:
        - JointControllerState 
    """
    global value_joint1
    value_joint1 = msg.process_value

def callback_joint2(msg):
    """
    Callback function for the joint position.
    
    Args:
        - JointControllerState 
    """
    global value_joint2
    value_joint2 = msg.process_value



def joint_action():
    # Create ROS node
    """"
    This function controls the commands passed to the joints. Joint 1 will rotate 360 degrees to scan the room."
    
    Raises: 
        - ROSInterruptException
    
    """
    

    rospy.init_node('joint_pose_modifier', anonymous = True)
    joint1_pose_pub = rospy.Publisher('/robot_patrolling_explorer/joint1_position_controller/command', Float64, queue_size=10)
    rospy.Subscriber("/robot_patrolling_explorer/joint1_position_controller/state", JointControllerState, callback_joint1)
    print("Init value of joint 1: ", value_joint1)
    #joint1_pose_pub.publish(-3.14)

    joint2_pose_pub = rospy.Publisher('/robot_patrolling_explorer/joint2_position_controller/command', Float64, queue_size=10)
    rospy.Subscriber("/robot_patrolling_explorer/joint2_position_controller/state", JointControllerState, callback_joint2)

    while value_joint1 > -3.099:
        print("Value of joint 1: ", value_joint1)
        joint1_pose_pub.publish(-3.14)
        joint2_pose_pub.publish(0.70)
        
    while value_joint1 < 2.399:
        print("Value of joint 1: ", value_joint1)
        joint1_pose_pub.publish(2.40)
        joint2_pose_pub.publish(0.70)
    

    while value_joint2 > -0.55:
        print("Value of joint 2: ", value_joint2)
        joint2_pose_pub.publish(-0.57)
        
    if value_joint2 <= -0.55:
        while value_joint1 > -1.59:
            print("Value of joint 1: ", value_joint1)
            joint1_pose_pub.publish(-1.60)

        
        
    
    joint1_pose_pub.publish(0.0)
    joint2_pose_pub.publish(0.0)
    rospy.sleep(10)
    #print("Final value of joint 1: ", value_joint1)


if __name__ == '__main__':
    try:
        joint_action()
    except rospy.ROSInterruptException:
        pass
  