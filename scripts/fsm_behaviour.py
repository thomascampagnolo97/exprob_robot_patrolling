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
    2. if a reachable room has not been visited for 20 seconds, the room become **URGENT** and the robot must visit it.

When the robot's battery is low, it goes in the E location, and wait for some times (recharging action in :mod:`battery` node) before to
start again with the above behaviour.

Publishes to:
    /world_battery_sync: a Boolean flag for synchronization reasons with the :mod:`battery` node. When the world is correctly loaded 
    the battery's functionality start its execution

Subscribes  to:
    /world_loading: a Boolean flag to communicate when the map is created correctly.
    /battery_signal: a Boolean flag to communicate when battery is low and when is fully charged
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

from exprob_robot_patrolling.srv import GetCoordinates



# States of the Finite State Machine
STATE_BUILD_WORLD = 'BUILD_WORLD'       # State where the environment is build using the ontology
STATE_NO_EMERGENCY = 'NO_EMERGENCY'     # State of non-urgency room, the robot only moves between the corridors
STATE_SURVEILLANCE = 'SURVEILLANCE'     # State of urgency room, the robot must visit the urgency locations for surveillance purposes
STATE_RECHARGING = 'RECHARGING'         # State where the robot recharges its battery

# Transitions of the Finite State Machine.
TRANS_WORLD_DONE = 'world_done'             # Transition that identifies the correct loading of the map
TRANS_WAITING_MAP = 'waiting_map'           # Transition to check if the map is uploaded correctly
TRANS_BATTERY_LOW = 'battery_low'           # Transition for the low battery charge. The robot needs to recharge
TRANS_BATTERY_CHARGED = 'battery_charged'   # Transition notifying full battery charge
TRANS_URGENT_ROOM = 'urgent_room'           # Transition for room surveillance
TRANS_NO_URGENT_ROOM = 'no_urgent_room'     # Transition to notify that there are not urgent rooms

# Global variables and flags
wait_time = 1.5         # Global variable for the sleeping time
battery_status = 1      # Battery flag, 1 means that it is fully charged
urgency_status = 0      # Flag to identify the urgent rooms
world_loaded = False    # World flag, False when the map is not loaded, True when map is loaded correctly (pub in world_generator_node)
wb_sync = 0             # World-battery synchronization flag for the battery_node
time_threshold = 7     # Time threshold to signal the urgency of the rooms

shared_location = ''    # List of the shared locations given by the connectedTo data property of the OWL
urgent_rooms = ''       # List from the query of the individuals in the URGENT class
robot_position = ''     # List that contains always one element, which is the robot position in that moment
timestamp = ''          # List of the queried timestamp

coordinates = {}        # Dictionary for the rooms coordinates
process_value=0

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


def go_to_coordinate(coor):
    """
    The :func:go_to_coordinate function moves the robot to the specified coordinates by calling the get_coordinate service. If the service returns that the target coordinates have been reached, the function returns 1. If the target coordinates have not been reached, the function calls itself again to try moving to the target coordinates again.

    Args:
        - coor: A string representing the key for the desired coordinates in the global dictionary coordinates.
    
    Returns:
        - response.return: An integer indicating whether the target coordinates have been reached (1) or not (0).

    """

    rospy.wait_for_service('/get_coordinate')
    get_coordinate = rospy.ServiceProxy('/get_coordinate', GetCoordinates)

    response = get_coordinate(coordinates[coor]['X'] , coordinates[coor]['Y'])
    print(response)
    if response.return_ == 1:
        print('Location reached')
    else:
        print('Location not reached')
        go_to_coordinate(coor)
    return response.return_


def setup_Dictionary():
    """
    This function initializes the global variable *coordinates* as a dictionary containing 
    the X and Y coordinates for each room in the list `rooms`. The function does this by calling 
    the ArmorClient and using the 'QUERY' command to retrieve the X and Y coordinates for each room 
    from the ontology. The function then adds the room's X and Y coordinates to the coordinates dictionary.

    """
    global coordinates

    client = ArmorClient("example", "ontoRef")

    rooms = ['R1', 'R2', 'R3', 'R4', 'C1', 'C2', 'E']
    
    for i in rooms:
        req=client.call('QUERY','DATAPROP','IND',['X_point', i])
        X=float(extract_value(req.queried_objects))
        req=client.call('QUERY','DATAPROP','IND',['Y_point', i])
        Y=float(extract_value(req.queried_objects))
        coordinates[i] = {'X': X, 'Y': Y}


def extract_value(input_list):
    """
    Function to rewrite the queried time stamp or coordinate, deleting the not digit part of the string, for both Rooms and Robot's data property

    Args
        - **list** of queried objects section of the Armor service message
    
    Returns
        all the element between the double quotes
    
    """
    # Extract the string value from the list
    value_string = input_list[0]
    # Remove the surrounding quotation marks and the xsd:float type indicator
    stripped_string = value_string[1:-1].split("^^")[0]
    # Remove any surrounding quotation marks
    stripped_string = stripped_string.strip('"')
    # Convert the string to a float and return it
    return float(stripped_string)


def callback_scan(msg):
    global process_value
    process_value = msg.process_value

def room_scan():
    """"
    This function controls the commands passed to the joints. Joint 1 will rotate 360 degrees to scan the room."
    
    """
    # Create a publisher for the cmd_vel topic
    print("Scanning..")
    joint1_pose_pub = rospy.Publisher('/robot_patrolling_explorer/joint1_position_controller/command', Float64, queue_size=10)
    rospy.Subscriber("/robot_patrolling_explorer/joint1_position_controller/state", JointControllerState, callback_scan)
    joint1_pose_pub.publish(-3.0)
    while process_value > -2.9:
        joint1_pose_pub.publish(-3.0)
    while process_value < 2.9:
        joint1_pose_pub.publish(3.0)
    joint1_pose_pub.publish(0.0)


def motion_control(self, helper, desPos):
        """
        Method used when the Robot has to change its position in the environment.
        
        This is the main function that define the motion of the robot in the map and takes the help of three other functions 
            - ``rooms_search``; 
            - ``path_planning``; 
            - ``time_set``.

        Args
            self: instance of the current class
            robPos: is the actual robot position when this function is called
            desPos: is the desired robot position
        
        Returns
            robot_position: the updated robot position at the end of the movement, when the location is changed

        """

        #global shared_location
        #global urgent_rooms
        #global robot_position

        self._helper = helper
        
        # client commands of the armor_api
        client = ArmorClient("example", "ontoRef")
        client.call('REASON','','',[''])
        
        # query the ontology about the position of the robot in the environment and extract the information with the function robot_location
        query_position=client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
        robot_position= self._helper.robot_location(query_position.queried_objects)
        
        # For debugging uncomment those lines
        #print('Robot position: ', self.robot_position) 
        #print('The robot changes position from: ', self.robot_position, 'to: ', desPos)

        # query the ontology about the robot's object property `canReach`
        query_reach_loc = client.call('QUERY','OBJECTPROP','IND',['canReach', 'Robot1'])
        robot_can_reach = self._helper.rooms_search(query_reach_loc.queried_objects)

        # For debugging uncomment this line
        #print('The locations that the robot can reach are: ', self.robot_can_reach) 


        if desPos in robot_can_reach:
            #print('Robot can reach the desired position!')
            # Update isIn property of the robot position
            client.call('REPLACE','OBJECTPROP','IND',['isIn', 'Robot1', desPos, robot_position])
            client.call('REASON','','',[''])
            
            # Update the timing information of the robot's data property `now`
            rob_time = client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
            old_rob_time = self._helper.time_set(rob_time.queried_objects)
            # compute the current time
            current_time=str(math.floor(time.time()))
            client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', current_time, old_rob_time])
            client.call('REASON','','',[''])
            
            # query the subclasses of the locations (ROOM, URGENT, CORRIDOR) from the ontology
            query_location = client.call('QUERY','CLASS','IND',[desPos, 'true'])
            subclass_location = query_location.queried_objects

            # For debugging uncomment the line below
            #print('The subclass of the location is: ', subclass_location)
            
            if subclass_location == ['URGENT'] or subclass_location == ['ROOM']:
                # Update the timing information of the room's data property `visitedAt`
                room_time = client.call('QUERY','DATAPROP','IND',['visitedAt', desPos])
                old_room_time = self._helper.time_set(room_time.queried_objects)
                # compute the current time
                current_time=str(math.floor(time.time()))
                client.call('REPLACE','DATAPROP','IND',['visitedAt', desPos, 'Long', current_time, old_room_time])
                client.call('REASON','','',[''])

        else:
            #print('Robot cannot reach the desired position!')
            query_desLoc_connection = client.call('QUERY','OBJECTPROP','IND',['connectedTo', desPos])
            # extract from the query the connected locations about the desired one
            des_location_connected = self._helper.rooms_search(query_desLoc_connection.queried_objects)
            
            # For debugging uncomment the line below
            #print('The desired location (desired robot position) is connected to: ', self.location_connected)

            common = self._helper.common_connection(des_location_connected, robot_can_reach)

            if common == None:
                # The are not common location between the connection of the actual robots location and the location connected to the desired one
                # The robot cannot reach the desired position
                client.call('REPLACE','OBJECTPROP','IND',['isIn', 'Robot1', robot_can_reach[0], robot_position])
                client.call('REASON','','',[''])

                # query the updated robots position in the environment and extract the information with the function robot_location
                query_position=client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
                robot_position= self._helper.robot_location(query_position.queried_objects)

                # query the ontology about the updated robot's object property `canReach`
                query_reach_loc = client.call('QUERY','OBJECTPROP','IND',['canReach', 'Robot1'])
                robot_can_reach = self._helper.rooms_search(query_reach_loc.queried_objects)

                common = self._helper.common_connection(des_location_connected, robot_can_reach)

            # updated the robots position in the environment with the common location
            client.call('REPLACE','OBJECTPROP','IND',['isIn', 'Robot1', common, robot_position])
            client.call('REASON','','',[''])

            # query the updated robots position in the environment and extract the information with the function robot_location
            query_position=client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
            robot_position=self._helper.robot_location(query_position.queried_objects)

            # updated the robots position in the environment with the desired location
            client.call('REPLACE','OBJECTPROP','IND',['isIn', 'Robot1', desPos, robot_position])
            client.call('REASON','','',[''])



            # Update the timing information of the robot's data property `now`
            rob_time = client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
            old_rob_time = self._helper.time_set(rob_time.queried_objects)
            # compute the current time
            current_time=str(math.floor(time.time()))
            client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', current_time, old_rob_time])
            client.call('REASON','','',[''])
            
            # query the subclasses of the locations (ROOM, URGENT, CORRIDOR) from the ontology
            query_locations = client.call('QUERY','CLASS','IND',[desPos, 'true'])
            subclass_location = query_locations.queried_objects

            # For debugging uncomment the line below
            #print('The subclass of the location is: ', subclass_location)
            
            if subclass_location == ['URGENT'] or subclass_location == ['ROOM']:
                # Update the timing information of the room's data property `visitedAt`
                room_time = client.call('QUERY','DATAPROP','IND',['visitedAt', desPos])
                old_room_time = self._helper.time_set(room_time.queried_objects)
                # compute the current time
                current_time=str(math.floor(time.time()))
                client.call('REPLACE','DATAPROP','IND',['visitedAt', desPos, 'Long', current_time, old_room_time])
                client.call('REASON','','',[''])


        # query the updated robots position in the environment and extract the information with the function robot_location
        query_position=client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
        robot_position=self._helper.robot_location(query_position.queried_objects)
        go_to_coordinate(robot_position)

        # For debugging uncomment the line below
        #print('The robot is in: ', self.robot_position)

        client.call('REASON','','',[''])


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
                                             TRANS_NO_URGENT_ROOM])
        
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

            setup_Dictionary()
            go_to_coordinate('E')

            # Reasoning OWL
            client.call('REASON','','',[''])

            # Output to visualize in the terminal the actual position of the robot
            # print(f"{bcolors.STATUS}Actually the robot is in: {bcolors.ENDC}", robot_position)

            
            return TRANS_WORLD_DONE

class No_emergency(smach.State):
    """
    Class that defines the ``NO_EMERGENCY`` state.
    The robot will stay in the corridors until any room becomes **URGENT**.

    The logic is:
        - the robot will start in one corridor;
        - if no room is URGENT, the robot stays in the corridor 3 seconds and change to the other corridor with the function ``motion_control`` in :mod:`state_machine_helper`
        - if one or more rooms becomes *URGENT*, the FSM goes to the state ``SURVEILLANCE``.

    **Important**: the state gets notified by the ``battery_callback`` in :mod:`battery` which is the callback of the topic ``/battery_signal`` with higher priority. 
    This interrupt any other action performed in the other states.
    
    Args
        smachState: State base interface
    
    Returns
        TRANS_BATTERY_LOW: transition that change status going to the ``RECHARGING``
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
                                             TRANS_NO_URGENT_ROOM])
        
        self._helper = helper

    def execute(self, userdata):

        global urgent_rooms
        global robot_position
        global time_threshold

        # Subscribers to the topic `battery_signal`
        rospy.Subscriber(anm.TOPIC_BATTERY_SIGNAL, Bool, self._helper.battery_callback) # battery flag (battery_status)

        # Publisher for the sync between world and battery
        pub_wb_sync = rospy.Publisher(anm.TOPIC_SYNC_WORLD_BATTERY, Bool, queue_size=10)
        

        #rospy.sleep(wait_time)

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

        # # Procedure for checking if there are some urgent rooms as long as the robot is in the corridors
        # locations_list = ['R1', 'R2', 'R3', 'R4']
                
        # for room in locations_list:
        #     # compute the current time
        #     current_time=str(math.floor(time.time()))
        #     # query the ontology about the room's object property `visitedAt` (time information), extract in old_room_rime
        #     room_time = client.call('QUERY','DATAPROP','IND',['visitedAt', room])
        #     old_room_time = self._helper.time_set(room_time.queried_objects)
            
        #     # cast the time values (string) to integer values and check if the threshold has been exceeded
        #     # if yes, the transition to the urgent rooms is returned
        #     if (int(current_time)-int(old_room_time)) > time_threshold:
        #         print(f"\n{bcolors.URGENT}Warning: There are some urgent rooms!{bcolors.ENDC}")
        #         # query the ontology about the robot's object property `now` (time information), extract in old_rob_time
        #         rob_time = client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
        #         old_rob_time = self._helper.time_set(rob_time.queried_objects)
        #         # compute the current time
        #         current_time=str(math.floor(time.time()))
        #         # replace the old_rob_time with the new value (current_time) of the robot's object property `now`
        #         client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', current_time, old_rob_time])
        #         # Reasoning OWL
        #         client.call('REASON','','',[''])
        #         # query the ontology about the room's object property `visitedAt` (time information), extract in old_room_rime
        #         room_time = client.call('QUERY','DATAPROP','IND',['visitedAt', room])
        #         old_room_time = self._helper.time_set(room_time.queried_objects)
        #         # Reasoning OWL
        #         client.call('REASON','','',[''])

        #         # list of the urgent rooms
        #         urgent_rooms = self._helper.urgency()
        #         # Output to visualize in the terminal the urgent rooms
        #         print(f"{bcolors.URGENT}The urgent rooms are: {bcolors.ENDC}", urgent_rooms)

        #         return TRANS_URGENT_ROOM

        # Main Behaviuor of the state
        if self._helper.battery_status == 1: # battery conditions check, that task has the higher priority
            if self._helper.urgency_status == 0 : # second condition, check if there are some urgent rooms
                if robot_position == 'C1':
                    print(f"{bcolors.MOVING}The Robot is in C1, should go in C2{bcolors.ENDC}\n")

                    #rospy.sleep(wait_time)

                    # call function to change the position of the robot (from actual position to the desired one)
                    motion_control(self, self._helper, 'C2')
                    client.call('REASON','','',[''])
                    
                    return TRANS_NO_URGENT_ROOM
                
                elif robot_position == "C2":
                    print(f"{bcolors.MOVING}The Robot is in C2, should go in C1{bcolors.ENDC}\n")

                    #rospy.sleep(wait_time)
                    
                    # call function to change the position of the robot (from actual position to the desired one)
                    motion_control(self, self._helper, 'C1')
                    client.call('REASON','','',[''])
                    
                    return TRANS_NO_URGENT_ROOM
                else:
                    motion_control(self, self._helper, 'C1')
                    return TRANS_NO_URGENT_ROOM                
            else:
                # there are some urgent rooms, go to the SURVEILLANCE state
                return TRANS_URGENT_ROOM
        else :
            # the battery charge is low, go to the RECHARGING state
            print(f"{bcolors.BATTERY}Warning: the battery charge is low!{bcolors.ENDC}\n")
            return TRANS_BATTERY_LOW

class Recharging(smach.State):
    """
    Class that defines the ``RECHARGING`` state.
    The battery charge is low, advertised by ``battery_callback`` in :mod:`battery`, which will modify the variable flag ``battery_status``.
    
    Args
        smachState: State base interface
    
    Returns
        TRANS_BATTERY_CHARGED: transition that changes status, going back to the ``NO_EMERGENCY`` state
        TRANS_BATTERY_LOW: loop transition that will keep the FSM active in this state as long as the battery becomes fully recharged

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
                                             TRANS_NO_URGENT_ROOM])
        
        self._helper = helper

    def execute(self, userdata):

        global robot_position

        # Subscribers to the topic `battery_signal`
        rospy.Subscriber(anm.TOPIC_BATTERY_SIGNAL, Bool, self._helper.battery_callback) # battery flag (battery_status)

        
        # client commands of the armor_api
        client = ArmorClient("example", "ontoRef")
        # Reasoning OWL
        client.call('REASON','','',[''])

        print("\n\n-------- Executing state RECHARGING --------\n")
        
        query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
        robot_position = self._helper.robot_location(query_position.queried_objects)

        #rospy.sleep(wait_time)

        # Main Behaviuor of the state
        if self._helper.battery_status == 0:
            print(f"{bcolors.BATTERY}Battery recharging ... {bcolors.ENDC}\n")
            # call function to change the position of the robot (from actual position to the recharging room, E)
            motion_control(self, self._helper, 'E')
            #client.call('REASON','','',[''])
            
            return TRANS_BATTERY_LOW
        else:
            print(f"{bcolors.BATTERY}Battery charging complete! {bcolors.ENDC}\n")
            # the battery charge is fully recharged, go to the NO_EMERGENCY state
            return TRANS_BATTERY_CHARGED

class Surveillance(smach.State):
    """
    Class that defines the ``SURVEILLANCE`` state. Only when the ``urgency_status`` variable is ``1`` the robot must visit the urgency rooms.
    It calls the ``motion_control`` function in :mod:`state_machine_helper` to move the robot and gets advertised by the ``battery_callback`` in :mod:`battery` in case of low battery.

    Args
        smachState: State base interface
    
    Returns
        TRANS_BATTERY_LOW: transition that change status to the ``RECHARGING`` state
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
                                             TRANS_NO_URGENT_ROOM])
        
        self._helper = helper
        
    def execute(self, userdata):

        global urgency_status
        global urgent_rooms
        global robot_position

        # Subscribers to the topic `battery_signal`
        rospy.Subscriber(anm.TOPIC_BATTERY_SIGNAL, Bool, self._helper.battery_callback) # battery flag (battery_status)

        # client commands of the armor_api
        client = ArmorClient("example", "ontoRef")
        # reasoning OWL
        client.call('REASON','','',[''])

        print("\n\n-------- Executing state SURVEILLANCE --------\n")

        # list of the urgent rooms
        urgent_rooms = self._helper.urgency()
        # Output to visualize in the terminal the urgent rooms
        print(f"{bcolors.URGENT}The urgent rooms are: {bcolors.ENDC}", urgent_rooms)

        #rospy.sleep(wait_time)
        
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
                    
                    #client.call('REASON','','',[''])
                    motion_control(self, self._helper, 'R1')
                    room_scan()
                    break
                
                elif "R2" in i:
                    #query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
                    #robot_position = self._helper.robot_location(query_position.queried_objects)
                    #client.call('REASON','','',[''])
                    motion_control(self, self._helper, 'R2')
                    room_scan()
                    #print(f"{bcolors.STATUS}The room surveillanced by the robot is: {bcolors.ENDC}", robot_position)
                    break
                
                elif "R3" in i:
                    # query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
                    # robot_position = self._helper.robot_location(query_position.queried_objects)
                    #client.call('REASON','','',[''])
                    motion_control(self, self._helper, 'R3')
                    room_scan()
                    #print(f"{bcolors.STATUS}The room surveillanced by the robot is: {bcolors.ENDC}", new_pose)
                    break
                
                elif "R4" in i:
                    # query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
                    # robot_position = self._helper.robot_location(query_position.queried_objects)
                    #client.call('REASON','','',[''])
                    motion_control(self, self._helper, 'R4')
                    room_scan()
                    #print(f"{bcolors.STATUS}The room surveillanced by the robot is: {bcolors.ENDC}", new_pose)
                    break
        
        return TRANS_URGENT_ROOM

def main():
    """
    Main function of the Finite State Machine that initializes the node ``fsm_behaviour`` using SMACH modules.
    This method create the FSM and specifies the states with the relative transitions.
    Since the behaviour of the battery has higher priority with respect to all the other states, here it's initialized
    the subscriber of the battery topic.

    """
    # Before to initialize the ROS node waits some time to allow the `world_generator` node to 
    # create the environment of the world from the ontology (OWL)
    #rospy.sleep(wait_time)
    # ROS Node initialization
    #rospy.init_node('fsm_behaviour')
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
                                             TRANS_NO_URGENT_ROOM : STATE_BUILD_WORLD})
        
        smach.StateMachine.add(STATE_NO_EMERGENCY, No_emergency(helper), 
                                transitions={TRANS_WORLD_DONE : STATE_NO_EMERGENCY, 
                                             TRANS_WAITING_MAP : STATE_NO_EMERGENCY,
                                             TRANS_BATTERY_LOW : STATE_RECHARGING,
                                             TRANS_BATTERY_CHARGED : STATE_NO_EMERGENCY,
                                             TRANS_URGENT_ROOM : STATE_SURVEILLANCE,
                                             TRANS_NO_URGENT_ROOM : STATE_NO_EMERGENCY})
        
        smach.StateMachine.add(STATE_RECHARGING, Recharging(helper), 
                                transitions={TRANS_WORLD_DONE : STATE_RECHARGING, 
                                             TRANS_WAITING_MAP : STATE_RECHARGING,
                                             TRANS_BATTERY_LOW : STATE_RECHARGING,
                                             TRANS_BATTERY_CHARGED : STATE_NO_EMERGENCY,
                                             TRANS_URGENT_ROOM : STATE_RECHARGING,
                                             TRANS_NO_URGENT_ROOM : STATE_RECHARGING})
        
        smach.StateMachine.add(STATE_SURVEILLANCE, Surveillance(helper), 
                                transitions={TRANS_WORLD_DONE : STATE_SURVEILLANCE, 
                                             TRANS_WAITING_MAP : STATE_SURVEILLANCE,
                                             TRANS_BATTERY_LOW : STATE_RECHARGING,
                                             TRANS_BATTERY_CHARGED : STATE_SURVEILLANCE,
                                             TRANS_URGENT_ROOM : STATE_SURVEILLANCE,
                                             TRANS_NO_URGENT_ROOM : STATE_NO_EMERGENCY})
    
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