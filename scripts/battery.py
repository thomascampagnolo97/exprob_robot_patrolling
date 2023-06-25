#!/usr/bin/env python
"""
.. module:: battery
    :platform: Unix
    :synopsis: Python code that descrive the behaviour of the battery. DIscharge and recharge cycles.

.. moduleauthor:: Thomas Campagnolo <s5343274@studenti.unige.it>

ROS node which defines the behavior of the robot battery. This node simulate quite realistically the battery discharge 
and charge cycles with the use of a counter variable ``battery_level`` and the time parameters ``time_discharge`` and ``time_recharge``.
If the battery is fully charged, the ``battery_status`` is published to ``1``. If instead the battery level is below the threshold 
then ``battery_status`` is ``0``, indicating that the battery is low on charge. For this reason, the :mod:`fsm_behaviour` changes its 
execution between all the others states to ``MOVE2RECHARGING`` and  ``RECHARGING`` states.

Publishes to:
    /battery_signal: a boolean flag to communicate when battery is low and when is totally charged

Subscribes  to:
    /world_battery_sync: a Boolean flag for synchronization reasons with the :mod:`fsm_behaviour` node. When the world is correctly loaded
    the battery's functionality start its execution.
    /start_recharging: a boolean flag used to indicate that the robot is in the recharging station and therefore the battery recharging cycle can begin.
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
time_recharge = 0.1     # recharge time of a single step
time_discharge = 5.0    # discharge time of a single step (first cycle)

sync_status = 0     # flag for sync with the world
sync_recharge = 0   # flag for sync with the correct robot position (the robot is in the recharging station, room E)

battery_level = anm.BATTERY_CAPACITY # initial and max battery capacity
threshold = 30   # minimum battery charge threshold, notify the low battery level


def sync_callback(data):
    """ 
    Callback function for the fsm publisher ``/world_battery_sync``, that modifies the value of the global variable ``sync_status`` and will let the code start. 
    
    """

    global sync_status

    if data.data == 0:
        sync_status = 0

    elif data.data == 1:
        sync_status = 1


def recharge_callback(data):
    """ 
    Callback function for the fsm publisher ``/start_recharging``, that modifies the value of the global variable ``sync_recharge`` 
    and will let the recharging cycle begins. 
    
    """

    global sync_recharge

    if data.data == 0:
        sync_recharge = 0

    elif data.data == 1:
        sync_recharge = 1



def battery_discharge(set_flag, level):
    """
    This function defines the battery discharge cycle. In general, the execution start from the maximum capacity (``BATTERY_CAPACITY``). 
    The level is decreased in steps of 2 for each discharge time parameter.
    If the level reaches values lower than or equal to the minimum charge ``threshold``, the ``battery_status`` is set to ``0``. 
    This indicates that the battery is low and the robot must move towards the charging station, room E.
    While moving towards this position, the discharge cycle parameters are modified in order to continue the discharge of the 
    battery starting from the current charge level and the discharge time is longer to allow the robot to reach room E to recharge 
    the battery (behavior like energy saving).
    If the synchronization flag with recharging ``sync_recharge`` is ``1`` this means that the robot has reached the recharging 
    station and therefore the discharge cycle is interrupted to allow the robot to recharge.


    Args:
        - set_flag: flag that sets the discharge mode, standard or energy saving
        - level: counter variable that simulates the dynamic level of the battery during the cycle

    Returns:
        - battery_status: the battery flag
        - level: actual battery level

    """

    rospy.Subscriber(anm.TOPIC_START_CHARGE_BATTERY, Bool, recharge_callback) 

    # General parameters for standard discharge cycle 
    if set_flag == 0:
        actual_battery_level = anm.BATTERY_CAPACITY
        threshold = 30
        time_discharge = 5.0
    # Energy saving parameters, the robot should reach the recharging station
    elif set_flag == 1:
        actual_battery_level = level
        threshold = 0
        time_discharge = 8.0

    for level in range(actual_battery_level, -2, -2):
        print("Battery level: ", level)

        rospy.sleep(time_discharge)
    
        if level <= threshold:
            battery_status = 0
            return battery_status, level
        
        if sync_recharge == 1:
            battery_status = 0
            return battery_status, level


def battery_recharge(level):
    """
    This function defines the battery recharge cycle. Starting from the min capacity reached in the discharge cycle, 
    the level is increased in steps of 2 for each recharge time parameter.
    When the level reach the maximum capacity (``BATTERY_CAPACITY``) value, the ``battery_status`` is set to ``1``. 
    This indicates that the battery is fully recharged.

    Args:
        level: counter variable that simulates the dynamic level of the battery during the cycle

    Returns:
        - battery_status: the battery flag
        - level: actual battery level
        
    """

    for level in range(level, anm.BATTERY_CAPACITY+1, 2):
        print("Battery level recharged: ", level)
        rospy.sleep(time_recharge)

        if level == anm.BATTERY_CAPACITY:
            battery_status = 1
            return battery_status, level


def reset_goal():
    """
    Function that cancels any existing goals using the Action client for the ``move_base`` action server
 
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

    rospy.Subscriber(anm.TOPIC_START_CHARGE_BATTERY, Bool, recharge_callback)    # subscriber world flag for sync


    while not rospy.is_shutdown():

        if sync_status == 1:
            # the world is loaded in the FSM, the battery behaviour starts its execution
            # The battery charge is higher than the min threshold

            set_discharge = 0 #standard discharge cycle
            battery = battery_discharge(set_discharge, battery_level)   # discharge cycle
            pub.publish(battery[0])

            if battery[0] == 0:
                # the battery level is low
                reset_goal()
                set_discharge = 1 # flag that change the discharge mode, set to energy saving
                battery = battery_discharge(set_discharge, battery[1])   # discharge cycle
                if sync_recharge == 1:
                    # the robot is in the recharging station, room E
                    battery = battery_recharge(battery[1]) # recharge cycle
                    pub.publish(battery[0])


        else:
            # the world is not yet loaded in the FSM
            print("Waiting map ... ")
            rospy.sleep(5)


if __name__ == '__main__':

    try:
        main_battery_behaviour()
    except rospy.ROSInterruptException:
        pass