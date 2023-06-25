#!/usr/bin/env python
"""
.. module:: fsm_behaviour
    :platform: Unix
    :synopsis: Python code of the finite state machine for the robot
    
.. moduleauthor:: Thomas Campagnolo <s5343274@studenti.unige.it>

**Overview**

The scenario involves a robot deployed in a indoor environment for surveillance purposes.
This is finite state machine that uses the `Smach tool <https://wiki.ros.org/smach>`_ to implement it in ROS. 

The FSM menages the surveillance behavior of the robot. It moves among locations with this policy:
    1. it should mainly stay on corridors,
    2. if a reachable room has not been visited for 20 seconds, the room becomes **URGENT** and the robot must visit it.

When the robot's battery is low, it cancel any goals and goes in the E location with an energy saving mode. Here it waits
for some times (recharging action in :mod:`battery` node) before to start again with the above behaviour.

Publishes to:
    - /world_battery_sync: a Boolean flag for synchronization reasons with the :mod:`battery` node. When the world is correctly loaded the battery's functionality start its execution
    - /scanning_room: a Boolean flag for the :mod:`room_scan` node. When the robot has reached the room it starts scanning the room for patrol purposes.
    - /start_recharging: a Boolean flag that notify the robot's position in the recharging station, room E

Subscribes  to:
    - /world_loading: a Boolean flag to communicate when the map is created correctly.
    - /battery_signal: a Boolean flag to communicate when battery is low and when is fully charged
"""

import rospkg
import rospy
import roslib
import os
import time
import math
import random
import smach
import smach_ros

from armor_api.armor_client import ArmorClient

from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import Twist
from control_msgs.msg import JointControllerState

# Import constant name defined to structure the architecture
from exprob_robot_patrolling import architecture_name_mapper as anm

# Import the class that support the interface of the Finite State Machine, which is available in this file
from exprob_robot_patrolling.state_machine_helper import Helper


# States of the Finite State Machine
STATE_BUILD_WORLD = 'BUILD_WORLD'           # State where the environment is build using the ontology
STATE_NO_EMERGENCY = 'NO_EMERGENCY'         # State of non-urgency room, the robot only moves between the corridors
STATE_SURVEILLANCE = 'SURVEILLANCE'         # State of urgency room, the robot must visit the urgency locations for surveillance purposes
STATE_MOVE2RECHARGING = 'MOVE2RECHARGING'   # State where the robot move towards the recharging station, room E
STATE_RECHARGING = 'RECHARGING'             # State where the robot recharges its battery

# Transitions of the Finite State Machine.
TRANS_WORLD_DONE = 'world_done'             # Transition that identifies the correct loading of the map
TRANS_WAITING_MAP = 'waiting_map'           # Transition to check if the map is uploaded correctly
TRANS_BATTERY_LOW = 'battery_low'           # Transition for the low battery charge. The robot needs to recharge, so it moves toward the room E
TRANS_BATTERY_CHARGED = 'battery_charged'   # Transition notifying full battery charge
TRANS_URGENT_ROOM = 'urgent_room'           # Transition for room surveillance
TRANS_NO_URGENT_ROOM = 'no_urgent_room'     # Transition to notify that there are not urgent rooms
TRANS_RECHARGE = 'battery_recharge'         # Transition for the battery recharge

# Global variables and flags
wait_time = 1.5         # Global variable for the sleeping time
battery_status = 1      # Battery flag, 1 means that it is fully charged
urgency_status = 0      # Flag to identify the urgent rooms
world_loaded = False    # World flag, False when the map is not loaded, True when map is loaded correctly (pub in world_generator_node)
wb_sync = 0             # World-battery synchronization flag for the battery_node
start_recharge = 0      # Recharge flag synchronization

urgent_rooms = ''       # List from the query of the individuals in the URGENT class
robot_position = ''     # List that contains always one element, which is the robot position in that moment


# Arguments for loading the ontology
rp = rospkg.RosPack()
assignment_path = rp.get_path('exprob_robot_patrolling')
WORLD_ONTOLOGY_FILE_PATH = os.path.join(assignment_path, "topological_map", "world_surveillance.owl") # also used for debugging the ontology
WEB_PATH = 'http://bnc/exp-rob-lab/2022-23'

# Colors for messages
class bcolors:
    BATTERY = '\033[96m'
    STATUS = '\033[92m'
    MOVING = '\033[93m'
    URGENT = '\033[91m'
    ENDC = '\033[0m'



def room_scan():
    """
    Function that controls the joint1 such that the robot can perform a scanning of the room with the camera. 
    Joint 1 will rotate to scan the room.
    To do this purpose, the movement of the robot arm is implemented in :mod:`room_scan`.
    
    """

    print("Room scanning...")

    # Create a publisher for the scan topic
    scan_pub = rospy.Publisher(anm.TOPIC_SCANNING_ROOM, Bool, queue_size=10)
    scan = 1
    scan_pub.publish(scan)
    rospy.sleep(3.0)
    scan = 0
    scan_pub.publish(scan)



class Build_world(smach.State):
    """
    Class that defines the ``BUILD_WORLD`` state, in which the code waits for the enviroment to be created.
    When the map is received and loaded the ``world_loaded`` boolean variable will be set to True, the robot will enter in 
    the Corridor and the outcome *TRANS_WORLD_DONE* will make the state end to switch to the next state ``NO_EMERGENCY``.

    Args
        smachState: State base interface
    
    Returns
        TRANS_WAITING_MAP: transition that will keep the FSM active in this state
        TRANS_WORLD_DONE: transition condition that will make this state end and go to the new next state ``NO_EMERGENCY``

    """

    def __init__(self, helper):
        """ 
		Method that initializes the state ``BUILD_WORLD``
		
		Args:
			self: instance of the current class

		"""

        smach.State.__init__(self, outcomes=[TRANS_WORLD_DONE, 
                                             TRANS_WAITING_MAP,
                                             TRANS_BATTERY_LOW,
                                             TRANS_BATTERY_CHARGED,
                                             TRANS_URGENT_ROOM,
                                             TRANS_NO_URGENT_ROOM,
                                             TRANS_RECHARGE])
        
        self._helper = helper
    
    def execute(self, userdata):

        global world_loaded
        global robot_position
        global pub_wb_sync


        # Subscriber of the world flag given by the world_generator_node
        rospy.Subscriber(anm.TOPIC_WORLD_LOAD, Bool, self._helper.world_callback)
        # Subscribers to the topic `battery_signal`
        rospy.Subscriber(anm.TOPIC_BATTERY_SIGNAL, Bool, self._helper.battery_callback) # battery flag (battery_status)

        # Publisher for the sync between world and battery
        pub_wb_sync = rospy.Publisher(anm.TOPIC_SYNC_WORLD_BATTERY, Bool, queue_size=10)
        

        #rospy.sleep(wait_time)

        client = ArmorClient("example", "ontoRef")

        print("\n\n-------- Executing state BUILD WORLD --------\n")

        # Main Behaviuor of the state
        # If the world is not created yet, the battery node doesn't start its execution and the FSM remains in this state
        if self._helper.world_loaded == False:
            print("Creating world ... \n")
            
            return TRANS_WAITING_MAP
        # if the world is created, the WORLD ONTOLOGY is loaded and start the sync with the battery node. The FSM goes to the next state.
        elif self._helper.world_loaded == True:
            print("World created!")
            
            # client commands of the armor_api
            client.call('LOAD','FILE','',[WORLD_ONTOLOGY_FILE_PATH, WEB_PATH, 'true', 'PELLET', 'false'])
            print('Map loaded FSM starts the plan!!!!')

            wb_sync = 1
            pub_wb_sync.publish(wb_sync)

            self._helper.room_dictionary()  # Get the X-Y coordinates of the rooms
            self._helper.go_to_coordinate(anm.INIT_LOCATION) # Move the robot to the starting position, room E

            # Reasoning OWL
            client.call('REASON','','',[''])

            
            return TRANS_WORLD_DONE

class No_emergency(smach.State):
    """
    Class that defines the ``NO_EMERGENCY`` state.
    The robot will stay in the corridors until any room becomes **URGENT**.

    The logic is:
        - the robot will start in one corridor;
        - if no room is URGENT, the robot stays in the corridor for some times and change to the other corridor with the function ``motion_control`` in :mod:`state_machine_helper`
        - if one or more rooms becomes *URGENT*, the FSM goes to the state ``SURVEILLANCE``.

    **Important**: the state gets notified by the ``battery_callback`` in :mod:`battery` which is the callback of the topic ``/battery_signal`` with higher priority. 
    This interrupt any other action performed in the other states.
    
    Args
        smachState: State base interface
    
    Returns
        TRANS_BATTERY_LOW: transition that change status going to the ``MOVE2RECHARGING``
        TRANS_URGENT_ROOM: transition that will make this state end and go to the ``SURVEILLANCE``
        TRANS_NO_URGENT_ROOM: loop transition that will keep the FSM active in this state
        
    """
    
    def __init__(self, helper):
        """ 
		Method that initializes the state ``NO_EMERGENCY``
		
		Args:
			self: instance of the current class

		"""

        smach.State.__init__(self, outcomes=[TRANS_WORLD_DONE, 
                                             TRANS_WAITING_MAP,
                                             TRANS_BATTERY_LOW,
                                             TRANS_BATTERY_CHARGED,
                                             TRANS_URGENT_ROOM,
                                             TRANS_NO_URGENT_ROOM,
                                             TRANS_RECHARGE])
        
        self._helper = helper

    def execute(self, userdata):

        global urgent_rooms
        global robot_position

        # Subscribers to the topic `battery_signal`
        rospy.Subscriber(anm.TOPIC_BATTERY_SIGNAL, Bool, self._helper.battery_callback) # battery flag (battery_status)

        # Publisher for the sync between world and battery
        pub_wb_sync = rospy.Publisher(anm.TOPIC_SYNC_WORLD_BATTERY, Bool, queue_size=10)
        
        print("\n\n-------- Executing state NO EMERGENCY --------\n")

        # client commands of the armor_api
        client = ArmorClient("example", "ontoRef")
        # Reasoning OWL
        client.call('REASON','','',[''])

        wb_sync = 1
        pub_wb_sync.publish(wb_sync)

        # Query from the ontology and find the actual robot position
        query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
        robot_position = self._helper.robot_location(query_position.queried_objects)
        # Output to visualize in the terminal the actual position of the robot
        print(f"{bcolors.STATUS}Actually the robot is in: {bcolors.ENDC}", robot_position)

        # list of the urgent rooms
        urgent_rooms = self._helper.urgency()
        # Output to visualize in the terminal the urgent rooms
        print(f"{bcolors.URGENT}The urgent rooms are: {bcolors.ENDC}", urgent_rooms)

        # Main Behaviuor of the state
        if self._helper.battery_status == 1: # battery conditions check, that task has the higher priority
            if self._helper.urgency_status == 0 : # second condition, check if there are some urgent rooms
                if robot_position == 'C1':
                    print(f"{bcolors.MOVING}The Robot is in C1, should go in C2{bcolors.ENDC}\n")

                    # call function to change the position of the robot (from actual position to the desired one)
                    self._helper.motion_control(self, 'C2')
                    client.call('REASON','','',[''])
                    
                    return TRANS_NO_URGENT_ROOM
                
                elif robot_position == "C2":
                    print(f"{bcolors.MOVING}The Robot is in C2, should go in C1{bcolors.ENDC}\n")
                    
                    # call function to change the position of the robot (from actual position to the desired one)
                    self._helper.motion_control(self, 'C1')
                    client.call('REASON','','',[''])
                    
                    return TRANS_NO_URGENT_ROOM
                else:
                    self._helper.motion_control(self, 'C1')
                    return TRANS_NO_URGENT_ROOM                
            else:
                # there are some urgent rooms, go to the SURVEILLANCE state
                return TRANS_URGENT_ROOM
        else :
            # the battery charge is low, go to the MOVE2RECHARGING state
            print(f"{bcolors.BATTERY}Warning: the battery charge is low!{bcolors.ENDC}\n")
            return TRANS_BATTERY_LOW

class Move2Recharging(smach.State):
    """
    Class that defines the ``MOVE2RECHARGING`` state.
    The battery charge is low, advertised by ``battery_callback`` in :mod:`battery`, which will modify the variable flag ``battery_status``.
    The robot will move towards the charging station.
    
    Args
        smachState: State base interface
    
    Returns
        TRANS_RECHARGE: transition that changes status, going to the ``RECHARGING`` state

    """
    
    def __init__(self, helper):
        """ 
		Method that initializes the state ``MOVE2RECHARGING``
		
		Args:
			self: instance of the current class

		"""

        smach.State.__init__(self, outcomes=[TRANS_WORLD_DONE, 
                                             TRANS_WAITING_MAP,
                                             TRANS_BATTERY_LOW,
                                             TRANS_BATTERY_CHARGED,
                                             TRANS_URGENT_ROOM,
                                             TRANS_NO_URGENT_ROOM,
                                             TRANS_RECHARGE])
        
        self._helper = helper

    def execute(self, userdata):

        global robot_position

        # Subscribers to the topic `battery_signal`
        rospy.Subscriber(anm.TOPIC_BATTERY_SIGNAL, Bool, self._helper.battery_callback) # battery flag (battery_status)
        
        # client commands of the armor_api
        client = ArmorClient("example", "ontoRef")
        # Reasoning OWL
        client.call('REASON','','',[''])

        print("\n\n-------- Executing state MOVE2RECHARGING --------\n")
        
        query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
        robot_position = self._helper.robot_location(query_position.queried_objects)

        # Main Behaviuor of the state
        if self._helper.battery_status == 0:
            print(f"{bcolors.BATTERY}Moving to the recharging room ... {bcolors.ENDC}\n")
            # call function to change the position of the robot (from actual position to the recharging room, E)
            self._helper.motion_control(anm.CHARGE_LOCATION)
            #client.call('REASON','','',[''])
            
            return TRANS_RECHARGE
        

class Recharging(smach.State):
    """
    Class that defines the ``RECHARGING`` state.
    The battery charge is low, advertised by ``battery_callback`` in :mod:`battery`, which will modify the variable flag ``battery_status``.
    If the robot is at the charging station (room E) the ``start_recharge`` flag is published in :mod:`battery` so that the node starts the 
    recharging cycle.
    
    Args
        smachState: State base interface
    
    Returns
        TRANS_BATTERY_CHARGED: transition that changes status, going back to the ``NO_EMERGENCY`` state
        TRANS_RECHARGE: loop transition that will keep the FSM active in this state as long as the battery becomes fully recharged

    """
    
    def __init__(self, helper):
        """ 
		Method that initializes the state ``RECHARGING``
		
		Args:
			self: instance of the current class

		"""

        smach.State.__init__(self, outcomes=[TRANS_WORLD_DONE, 
                                             TRANS_WAITING_MAP,
                                             TRANS_BATTERY_LOW,
                                             TRANS_BATTERY_CHARGED,
                                             TRANS_URGENT_ROOM,
                                             TRANS_NO_URGENT_ROOM,
                                             TRANS_RECHARGE])
        
        self._helper = helper

    def execute(self, userdata):

        global robot_position
        global start_recharge

        # Subscribers to the topic `battery_signal`
        rospy.Subscriber(anm.TOPIC_BATTERY_SIGNAL, Bool, self._helper.battery_callback) # battery flag (battery_status)

        # Publisher to notify the correct robot position in the recharging room
        pub_recharge = rospy.Publisher(anm.TOPIC_START_CHARGE_BATTERY, Bool, queue_size=10)
        
        # client commands of the armor_api
        client = ArmorClient("example", "ontoRef")
        # Reasoning OWL
        client.call('REASON','','',[''])

        print("\n\n-------- Executing state RECHARGING --------\n")
        
        query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
        robot_position = self._helper.robot_location(query_position.queried_objects)

        # Main Behaviuor of the state
        if robot_position == anm.CHARGE_LOCATION:
            if self._helper.battery_status == 0:
                print(f"{bcolors.BATTERY}Battery recharging ... {bcolors.ENDC}\n")
                # publish the flag to start the recharging cycle in battery node 
                start_recharge = 1
                pub_recharge.publish(start_recharge)
            
                return TRANS_RECHARGE

            else:
                print(f"{bcolors.BATTERY}Battery charging complete! {bcolors.ENDC}\n")
                # the battery charge is fully recharged, go to the NO_EMERGENCY state
                start_recharge = 0
                pub_recharge.publish(start_recharge)

                return TRANS_BATTERY_CHARGED
            

class Surveillance(smach.State):
    """
    Class that defines the ``SURVEILLANCE`` state. Only when the ``urgency_status`` variable is ``1`` the robot must visit the urgency rooms.
    It calls the ``motion_control`` function in :mod:`state_machine_helper` to move the robot and gets advertised by the ``battery_callback`` 
    in :mod:`battery` in case of low battery.

    Args
        smachState: State base interface
    
    Returns
        TRANS_BATTERY_LOW: transition that change status to the ``MOVE2RECHARGING`` state
        TRANS_NO_URGENT_ROOM: transition that specifies that no other room are urgent anymore and changes status to ``NO_EMERGENCY`` state
        TRANS_URGENT_ROOM: loop transition that will keep the FSM active in this state
    """
    
    
    def __init__(self, helper):
        """ 
		Method that initializes the state ``SURVEILLANCE``
		
		Args:
			self: instance of the current class

		"""

        smach.State.__init__(self, outcomes=[TRANS_WORLD_DONE, 
                                             TRANS_WAITING_MAP,
                                             TRANS_BATTERY_LOW,
                                             TRANS_BATTERY_CHARGED,
                                             TRANS_URGENT_ROOM,
                                             TRANS_NO_URGENT_ROOM,
                                             TRANS_RECHARGE])
        
        self._helper = helper
        
    def execute(self, userdata):

        global urgency_status
        global urgent_rooms
        global robot_position

        # Subscribers to the topic `battery_signal`
        rospy.Subscriber(anm.TOPIC_BATTERY_SIGNAL, Bool, self._helper.battery_callback) # battery flag (battery_status)

        scan_room_pub = rospy.Publisher(anm.TOPIC_SCANNING_ROOM, Bool, queue_size=10)

        # client commands of the armor_api
        client = ArmorClient("example", "ontoRef")
        # reasoning OWL
        client.call('REASON','','',[''])

        print("\n\n-------- Executing state SURVEILLANCE --------\n")

        # list of the urgent rooms
        urgent_rooms = self._helper.urgency()
        # Output to visualize in the terminal the urgent rooms
        print(f"{bcolors.URGENT}The urgent rooms are: {bcolors.ENDC}", urgent_rooms)
        
        # Main Behaviuor of the state
        if self._helper.battery_status == 0:  
            # the battery charge is low
            print(f"{bcolors.BATTERY}Warning: the battery charge is low!{bcolors.ENDC}")
            return TRANS_BATTERY_LOW
        
        elif self._helper.urgency_status == 0:
            # there are not urgent rooms 
            return TRANS_NO_URGENT_ROOM
        
        elif self._helper.urgency_status == 1:
            # there are some urgent rooms
            for i in urgent_rooms :
                # check which are the urgent rooms in the list that must be surveillanced by the robot
                query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
                robot_position = self._helper.robot_location(query_position.queried_objects)
                print(f"{bcolors.STATUS}The room surveillanced by the robot is: {bcolors.ENDC}", robot_position)

                if "R1" in i:
                    self._helper.motion_control('R1')
                    # Function to scan the room with the camera
                    room_scan()
                    break
                
                elif "R2" in i:
                    self._helper.motion_control('R2')
                    # Function to scan the room with the camera
                    room_scan()
                    break
                
                elif "R3" in i:
                    self._helper.motion_control('R3')
                    # Function to scan the room with the camera
                    room_scan()
                    break
                
                elif "R4" in i:
                    self._helper.motion_control('R4')
                    # Function to scan the room with the camera
                    room_scan()
                    break
        
        return TRANS_URGENT_ROOM

def main():
    """
    Main function of the Finite State Machine that initializes the node ``fsm_behaviour`` using SMACH modules.
    This method create the FSM and specifies the states with the relative transitions.
    Since the behaviour of the battery has higher priority with respect to all the other states, here it's initialized
    the subscriber of the battery topic.

    """
   
    rospy.init_node(anm.NODE_STATE_MACHINE, log_level=rospy.INFO)
    
    helper = Helper()
    
    # Create the Finite State Machine
    sm = smach.StateMachine(outcomes=['interface'])
    
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add(STATE_BUILD_WORLD, Build_world(helper), 
                                transitions={TRANS_WORLD_DONE : STATE_NO_EMERGENCY, 
                                             TRANS_WAITING_MAP : STATE_BUILD_WORLD,
                                             TRANS_BATTERY_LOW : STATE_BUILD_WORLD,
                                             TRANS_BATTERY_CHARGED : STATE_BUILD_WORLD,
                                             TRANS_URGENT_ROOM : STATE_BUILD_WORLD,
                                             TRANS_NO_URGENT_ROOM : STATE_BUILD_WORLD,
                                             TRANS_RECHARGE : STATE_BUILD_WORLD})
        
        smach.StateMachine.add(STATE_NO_EMERGENCY, No_emergency(helper), 
                                transitions={TRANS_WORLD_DONE : STATE_NO_EMERGENCY, 
                                             TRANS_WAITING_MAP : STATE_NO_EMERGENCY,
                                             TRANS_BATTERY_LOW : STATE_MOVE2RECHARGING,
                                             TRANS_BATTERY_CHARGED : STATE_NO_EMERGENCY,
                                             TRANS_URGENT_ROOM : STATE_SURVEILLANCE,
                                             TRANS_NO_URGENT_ROOM : STATE_NO_EMERGENCY,
                                             TRANS_RECHARGE : STATE_NO_EMERGENCY})
        
        smach.StateMachine.add(STATE_MOVE2RECHARGING, Move2Recharging(helper), 
                                transitions={TRANS_WORLD_DONE : STATE_MOVE2RECHARGING, 
                                             TRANS_WAITING_MAP : STATE_MOVE2RECHARGING,
                                             TRANS_BATTERY_LOW : STATE_MOVE2RECHARGING,
                                             TRANS_BATTERY_CHARGED : STATE_MOVE2RECHARGING,
                                             TRANS_URGENT_ROOM : STATE_MOVE2RECHARGING,
                                             TRANS_NO_URGENT_ROOM : STATE_MOVE2RECHARGING,
                                             TRANS_RECHARGE : STATE_RECHARGING})
        
        smach.StateMachine.add(STATE_RECHARGING, Recharging(helper), 
                                transitions={TRANS_WORLD_DONE : STATE_RECHARGING, 
                                             TRANS_WAITING_MAP : STATE_RECHARGING,
                                             TRANS_BATTERY_LOW : STATE_RECHARGING,
                                             TRANS_BATTERY_CHARGED : STATE_NO_EMERGENCY,
                                             TRANS_URGENT_ROOM : STATE_RECHARGING,
                                             TRANS_NO_URGENT_ROOM : STATE_RECHARGING,
                                             TRANS_RECHARGE : STATE_RECHARGING})
        
        smach.StateMachine.add(STATE_SURVEILLANCE, Surveillance(helper), 
                                transitions={TRANS_WORLD_DONE : STATE_SURVEILLANCE, 
                                             TRANS_WAITING_MAP : STATE_SURVEILLANCE,
                                             TRANS_BATTERY_LOW : STATE_MOVE2RECHARGING,
                                             TRANS_BATTERY_CHARGED : STATE_SURVEILLANCE,
                                             TRANS_URGENT_ROOM : STATE_SURVEILLANCE,
                                             TRANS_NO_URGENT_ROOM : STATE_NO_EMERGENCY,
                                             TRANS_RECHARGE : STATE_SURVEILLANCE})
    
    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__': 
    main()