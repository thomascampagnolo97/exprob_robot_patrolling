#!/usr/bin/env python
"""
.. module:: battery
    :platform: Unix
    :synopsis: Python code to change the battery level

.. moduleauthor:: Thomas Campagnolo <s5343274@studenti.unige.it>

ROS node which defines the behavior of the robot battery. This node simulate quite realistically the battery discharge 
and charge cycles with the use of a counter variable ``battery_level`` and the time parameters ``time_discharge`` and ``time_recharge``.
If the battery is fully charged, the ``battery_status`` is published to ``1``. If instead the battery level is below the minimum threshold 
then ``battery_status`` is ``0``. Depending on this, the :mod:`fsm_behaviour` changes its execution between ``RECHARGING`` state and all the others states.

Publishes to:
    /battery_signal: a boolean flag to communicate when battery is low and when is totally charged

Subscribes  to:
    /world_battery_sync: a Boolean flag for synchronization reasons with the :mod:`fsm_behaviour` node. When the world is correctly loaded
    the battery's functionality start its execution.
"""

import rospy
import actionlib
import random
from std_msgs.msg import Bool

from armor_api.armor_client import ArmorClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool

# Import constant name defined to structure the architecture
from exprob_robot_patrolling import architecture_name_mapper as anm

# Global parameters that define time informations
time_recharge = 0.7     # recharge time of a single step
time_discharge = 1.5    # discharge time of a single step

sync_status = 0  # flag for sync with the world

battery_level = anm.BATTERY_CAPACITY # initial and max battery capacity
threshold = 4   # minimum battery charge threshold


def sync_callback(data):
    """ 
    Callback function for the fsm publisher ``/world_battery_sync``, that modifies the value of the global variable ``sync_status`` and will let the code start. 
    
    """

    global sync_status

    if data.data == 0:
        sync_status = 0

    elif data.data == 1:
        sync_status = 1


def battery_discharge(level):
    """
    This function defines the battery discharge cycle. Starting from the maximum capacity (``BATTERY_CAPACITY``), 
    the level is decreased in steps of 2 for each discharge time parameter.
    If the level reaches values lower than or equal to the minimum charge ``threshold``, the ``battery_status`` is set to ``0``. 
    This indicates that the battery is low and must be recharged.

    Args:
        level: counter variable that simulates the dynamic level of the battery during the cycle

    Returns:
        battery_status: the battery flag

    """

    for level in range(anm.BATTERY_CAPACITY, -2, -2):
        print("Battery level: ", level)

        rospy.sleep(time_discharge)
    
        if level <= threshold:
            battery_status = 0
            return battery_status


def battery_recharge(level):
    """
    This function defines the battery recharge cycle. Starting from the min capacity reached in the discharge cycle, 
    the level is increased in steps of 2 for each recharge time parameter.
    When the level reach the maximum capacity (``BATTERY_CAPACITY``) value, the ``battery_status`` is set to ``1``. 
    This indicates that the battery is fully recharged.

    Args:
        level: counter variable that simulates the dynamic level of the battery during the cycle

    Returns:
        battery_status: the battery flag
        
    """

    for level in range(threshold, anm.BATTERY_CAPACITY+1, 2):
        print("Battery level recharged: ", level)
        rospy.sleep(time_recharge)

        if level == anm.BATTERY_CAPACITY:
            battery_status = 1
            return battery_status


def reset_goal():
    """
    Cancels any existing goals using the Action client for the `move_base` action server
 
    """
    # Action client
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    # Wait for the action server 
    client.wait_for_server()
    client.cancel_all_goals()


def main_battery_behaviour():
    """
    Function that define the battery behaviour with the initialization of the battery node.
    For synchronization with the FSM, this funcionality start its execution when :mod:`world_callback` returns ``1``, advertised by :mod:`fsm_behaviour` node.
    The boolean value of the battery to the state ``battery_status`` is advertised by :mod:`battery_discharge` and :mod:`battery_recharge`,
    and published in the dedicated topic.
    
    """

    rospy.init_node(anm.NODE_ROBOT_BATTERY, log_level=rospy.INFO)

    rospy.Subscriber(anm.TOPIC_SYNC_WORLD_BATTERY, Bool, sync_callback)    # subscriber world flag for sync
    pub = rospy.Publisher(anm.TOPIC_BATTERY_SIGNAL, Bool, queue_size=10)    # publisher of the battery status flag
    
    while not rospy.is_shutdown():

        if sync_status == 1:
            # the world is loaded in the FSM, the battery behaviour starts its execution
            # The battery charge is higher than the min threshold 
            battery_status = battery_discharge(battery_level)   # discharge cycle
            pub.publish(battery_status)

            if battery_status == 0:
                reset_goal()
                # the battery level is low
                battery_status = battery_recharge(battery_level) # recharge cycle
                pub.publish(battery_status)


        else:
            # the world is not yet loaded in the FSM
            print("Waiting map ... ")
            rospy.sleep(5)


if __name__ == '__main__':

    try:
        main_battery_behaviour()
    except rospy.ROSInterruptException:
        pass
