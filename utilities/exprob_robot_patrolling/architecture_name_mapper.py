#!/usr/bin/env python

import rospy


# Define the location in which the robot starts
INIT_LOCATION = 'E'

# Define the location in which the robot goes to recharge itself
CHARGE_LOCATION = 'E'
# ---------------------------------------------------------


# The name of the node representing the knowledge required for this scenario
NODE_STATE_MACHINE = 'fsm_behaviour'
# ---------------------------------------------------------


# The name of the world generator node
NODE_WORLD_GENERATOR = 'world_generator_node'

# The name of the topic solving the world generation
TOPIC_WORLD_LOAD = 'world_loading'
# -------------------------------------------------


# Name of the node representing the behaviour of the battery, required for this scenario
NODE_ROBOT_BATTERY = 'battery_node'

# Name of the topic where the battery signal is published
TOPIC_BATTERY_SIGNAL = 'battery_signal'

# Name of the topic for synchronization between world and battery initial execution
TOPIC_SYNC_WORLD_BATTERY = 'world_battery_sync'

# Name of the topic that tells the starting execution of the recharging cycle
TOPIC_START_CHARGE_BATTERY = 'start_recharging'

# Define the initial and max charge capacity of the battery
BATTERY_CAPACITY = 100
# ---------------------------------------------------------


# Name of the node of the robot motion
NODE_ROBOT_MOTION = 'robot_motion'

# Name of the service for robot motion
SERVICE_SET_GOAL_COORDINATES = 'set_goal_coordinates'
# ---------------------------------------------------------


# Name of the node of the robot motion
NODE_JOINTS_CONTROL = 'joint_pose_controller'

# Name of the topic for joint 1 command
TOPIC_JOINT1_COMMAND = '/robot_patrolling_explorer/joint1_position_controller/command'
# Name of the topic for joint 2 command
TOPIC_JOINT2_COMMAND = '/robot_patrolling_explorer/joint2_position_controller/command'
# Name of the topic for joint 3 command
TOPIC_JOINT3_COMMAND = '/robot_patrolling_explorer/joint3_position_controller/command'

# Name of the topic for joint 1 control state
TOPIC_JOINT1_STATE = '/robot_patrolling_explorer/joint1_position_controller/state'
# Name of the topic for joint 2 control state
TOPIC_JOINT2_STATE = '/robot_patrolling_explorer/joint2_position_controller/state'
# Name of the topic for joint 3 control state
TOPIC_JOINT3_STATE = '/robot_patrolling_explorer/joint3_position_controller/state'
# ---------------------------------------------------------


# Name of the node of the robot motion
NODE_ROOM_SCAN = 'room_scan_node'

# Name of the topic to perform the room scanning
TOPIC_SCANNING_ROOM = 'scanning_room'
# ---------------------------------------------------------


# Function used to label each log with a producer tag.
def tag_log(msg, producer_tag):
    return '@%s>> %s' % (producer_tag, msg)