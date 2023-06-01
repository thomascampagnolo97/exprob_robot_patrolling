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

# Define the initial and max charge capacity of the battery
BATTERY_CAPACITY = 40
# ---------------------------------------------------------


# Function used to label each log with a producer tag.
def tag_log(msg, producer_tag):
    return '@%s>> %s' % (producer_tag, msg)