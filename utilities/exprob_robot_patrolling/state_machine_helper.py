#!/usr/bin/env python
"""
.. module:: state_machine_helper
    :platform: Unix
	:synopsis: Python module for the Helper of the State Machine
   
.. moduleauthor:: Thomas Campagnolo <s5343274@studenti.unige.it>

ROS node in support of the main node :mod:`fsm_behaviour`. The software architecture allows initializing a 
helper class for the Final State Machine which controls the behavior of a surveillance robot. 
This node allows to have cleaner and more readable code in the ``fsm_behaviour.py`` node, in fact, every task
called in the previously mentioned code is defined in the current code.
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
from std_msgs.msg import Bool

from exprob_robot_patrolling.srv import GetCoordinates


class Helper:
    """
    This class is created to decouple the implementation of the Finite State Machine, allowing to have a more readable and cleaner code in the ``fsm_behaviour.py`` node.
    """
    def __init__(self):
        """
        Function that initializes the class Helper.
		
		Args:
			self: instance of the current class.
        
        """

        # Global variables and flags
        self.wait_time = 1.5         # Global variable for the sleeping time
        self.battery_status = 1      # Battery flag, 1 means that it is fully charged
        self.urgency_status = 0      # Flag to identify the urgent rooms
        self.world_loaded = False    # World flag, False when the map is not loaded, True when map is loaded correctly (pub in world_generator_node)

        self.shared_location = ''    # List of the shared locations given by the connectedTo data property of the OWL
        self.urgent_rooms = ''       # List from the query of the individuals in the URGENT class
        self.robot_position = ''     # List that contains always one element, which is the robot position in that moment
        self.timestamp = ''          # List of the queried timestamp

        self.coordinates = {}        # Dictionary for the rooms coordinates
        self.process_value = 0


    def world_callback(self, data):
        """ 
        Callback function for the map publisher ``/world_loading``, that modifies the value of the global variable ``world_loaded`` and will let the code start.

        Args:
			self: instance of the current class
            data: data in the topic ``/world_loading``

        Returns:
			self.world_loaded: Bool value that states the flag of the generation of the world
        
        """

        #global world_loaded
        if data.data == False:
            self.world_loaded = False

        elif data.data == True:
            self.world_loaded = True
        
        return self.world_loaded
    
    def battery_callback(self, data):
        """ 
        Callback function for the map publisher ``/battery_signal``, that modifies the value of the global variable ``battery_status`` and will let the code start.

        Args:
			self: instance of the current class
            data: data in the topic ``/battery_signal``

        Returns:
			self.battery_status: Bool value that states the flag about the battery status
        
        """
        #global battery_status
        if data.data == 0:
            self.battery_status = 0

        elif data.data == 1:
            self.battery_status = 1

        return self.battery_status
    
    def path_planning(self, listNow, listDes, robPos):
        """
        This method is needed to plan the path from one position to another, recursively scanning the list of reachable point for the robot 
        and the connected area to the desired position.

        Args
            self: instance of the current class
            l1: the list of string from querying the *connectedTo* data property of the actual robot position
            l2: the list of string from querying the *connectedTo* data property of the desired robot location
            robPos: the actual robot position in the moment the function is called for checking operation
        
        Returns
            self.shared_location: the shared location between the two lists

        """

        #global shared_location

        for i in listNow:
            for j in listDes:
                if i == j :
                    self.shared_location = i
                elif robPos == j:
                    self.shared_location = robPos
                else:
                    self.shared_location = ''

        return self.shared_location

    def time_set(self, timeList):
        """
        Method to rewrite the queried timestamp for both Rooms and Robot's data property.

        Args
            self: instance of the current class
            time_list: queried objects section of the Armor service message
        
        Returns
            self.timestamp: the integer value between the double quotes of the OWL, saved as list of string
        
        """
        self.timestamp = ''
        for i in timeList:
            for element in range(1, 11):
                self.timestamp = self.timestamp+i[element]

        return self.timestamp

    def rooms_search(self, roomList):
        """
        Method to rewrite the queried data property list of the room, saving in the returned list as separate strings

        Args
            self: instance of the current class
            roomList: queried objects section of the Armor service message

        Returns
            self.location_list: the list of strings of locations
        
        """
        self.location_list = []
        for i in roomList:
            
            if "R1" in i:
                self.location_list.append('R1') # room 1
            elif "R2" in i:
                self.location_list.append('R2') # room 2
            elif "R3" in i:
                self.location_list.append('R3') # room 3
            elif "R4" in i:
                self.location_list.append('R4') # room 4
            elif "C1" in i:
                self.location_list.append('C1') # corridor 1
            elif "C2" in i:
                self.location_list.append('C2') # corridor 2
            elif "E" in i:
                self.location_list.append('E') # special room for recharging

        return self.location_list

    def robot_location(self, isInList):
        """
        Method to extract the element of the queried ``isIn`` robot's object property.

        Args
            self: instance of the current class
            isInList: queried objects section of the Armor service message
                    
        """
        for i in isInList:
            if "R1" in i:
                return 'R1'
            elif "R2" in i:
                return'R2'
            elif "R3" in i:
                return'R3'
            elif "R4" in i:
                return'R4'
            elif "C1" in i:
                return'C1'
            elif "C2" in i:
                return'C2'
            elif "E" in i:
                return'E'
            
    def set_urgency(self):
        """
        Method that set to ``1`` (True) the urgency flag if there are some rooms in the ``urgency_rooms`' list,
        otherwise the flag will be set to ``0``.

        Args
            self: instance of the current class

        Returns
            self.urgency_status: Bool value that states the flag about the urgency status
            
        """

        if self.urgent_rooms == []:
            self.urgency_status = 0
        else:
            self.urgency_status = 1
        
        return self.urgency_status

    def urgency(self):
        """
        Method that recursively checks on the ``urgent_rooms`` list if there are some ``URGENT`` rooms.
        If the list is empty the flag ``urgency_status`` is set to ``0`` (False), otherwise if there are some urgent rooms the flag will be set to ``1``.

        Args
            self: instance of the current class

        Returns
            self.urgent_rooms: list of the urgent rooms, modified in the :mod:`Surveillance` status.

        """

        #global urgency_status
        #global urgent_rooms

        client = ArmorClient("example", "ontoRef")
        client.call('REASON','','',[''])
        
        # query the ontology to check if there are some URGENT instances
        self.query_urgent = client.call('QUERY','IND','CLASS',['URGENT'])
        self.urgent_rooms = self.rooms_search(self.query_urgent.queried_objects) 
        random.shuffle(self.urgent_rooms)

        # set the urgency flag
        self.set_urgency()
            
        return self.urgent_rooms

    def go_to_coordinate(self, coor):
        """
        The :func:go_to_coordinate function moves the robot to the specified coordinates by calling the get_coordinate service. If the service returns that the target coordinates have been reached, the function returns 1. If the target coordinates have not been reached, the function calls itself again to try moving to the target coordinates again.

        Args:
            - coor: A string representing the key for the desired coordinates in the global dictionary coordinates.
        
        Returns:
            - response.return: An integer indicating whether the target coordinates have been reached (1) or not (0).

        """

        get_coordinate = rospy.ServiceProxy('/get_coordinate', GetCoordinates)
        rospy.wait_for_service('/get_coordinate')

        response = get_coordinate(self.coordinates[coor]['X'] , self.coordinates[coor]['Y'])
        if response.return_ == 1:
            print('Location reached')
        else:
            print('Location not reached')
            self.go_to_coordinate(coor)
        return response.return_

    def common_connection(self, list1, list2):
        for self.string in list1:
            if self.string in list2:
                return self.string

    def motion_control(self, desPos):
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
        
        # client commands of the armor_api
        client = ArmorClient("example", "ontoRef")
        client.call('REASON','','',[''])
        
        # query the ontology about the position of the robot in the environment and extract the information with the function robot_location
        self.query_position=client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
        self.robot_position= self.robot_location(self.query_position.queried_objects)
        
        # For debugging uncomment those lines
        #print('Robot position: ', self.robot_position) 
        #print('The robot changes position from: ', self.robot_position, 'to: ', desPos)

        # query the ontology about the robot's object property `canReach`
        self.query_reach_loc = client.call('QUERY','OBJECTPROP','IND',['canReach', 'Robot1'])
        self.robot_can_reach = self.rooms_search(self.query_reach_loc.queried_objects)

        # For debugging uncomment this line
        #print('The locations that the robot can reach are: ', self.robot_can_reach) 


        if desPos in self.robot_can_reach:
            #print('Robot can reach the desired position!')
            # Update isIn property of the robot position
            client.call('REPLACE','OBJECTPROP','IND',['isIn', 'Robot1', desPos, self.robot_position])
            client.call('REASON','','',[''])
            
            # Update the timing information of the robot's data property `now`
            self.rob_time = client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
            self.old_rob_time = self.time_set(self.rob_time.queried_objects)
            # compute the current time
            self.current_time=str(math.floor(time.time()))
            client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', self.current_time, self.old_rob_time])
            client.call('REASON','','',[''])
            
            # query the subclasses of the locations (ROOM, URGENT, CORRIDOR) from the ontology
            self.query_location = client.call('QUERY','CLASS','IND',[desPos, 'true'])
            self.subclass_location = self.query_location.queried_objects

            # For debugging uncomment the line below
            #print('The subclass of the location is: ', subclass_location)
            
            if self.subclass_location == ['URGENT'] or self.subclass_location == ['ROOM']:
                # Update the timing information of the room's data property `visitedAt`
                self.room_time = client.call('QUERY','DATAPROP','IND',['visitedAt', desPos])
                self.old_room_time = self.time_set(self.room_time.queried_objects)
                # compute the current time
                self.current_time=str(math.floor(time.time()))
                client.call('REPLACE','DATAPROP','IND',['visitedAt', desPos, 'Long', self.current_time, self.old_room_time])
                client.call('REASON','','',[''])

        else:
            #print('Robot cannot reach the desired position!')
            self.query_desLoc_connection = client.call('QUERY','OBJECTPROP','IND',['connectedTo', desPos])
            # extract from the query the connected locations about the desired one
            self.des_location_connected = self.rooms_search(self.query_desLoc_connection.queried_objects)
            
            # For debugging uncomment the line below
            #print('The desired location (desired robot position) is connected to: ', self.location_connected)

            self.common = self.common_connection(self.des_location_connected, self.robot_can_reach)

            if self.common == None:
                # The are not common location between the connection of the actual robots location and the location connected to the desired one
                # The robot cannot reach the desired position
                client.call('REPLACE','OBJECTPROP','IND',['isIn', 'Robot1', self.robot_can_reach[0], self.robot_position])
                client.call('REASON','','',[''])

                # query the updated robots position in the environment and extract the information with the function robot_location
                self.query_position=client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
                self.robot_position= self.robot_location(self.query_position.queried_objects)

                # query the ontology about the updated robot's object property `canReach`
                self.query_reach_loc = client.call('QUERY','OBJECTPROP','IND',['canReach', 'Robot1'])
                self.robot_can_reach = self.rooms_search(self.query_reach_loc.queried_objects)

                self.common = self.common_connection(self.des_location_connected, self.robot_can_reach)

            # updated the robots position in the environment with the common location
            client.call('REPLACE','OBJECTPROP','IND',['isIn', 'Robot1', self.common, self.robot_position])
            client.call('REASON','','',[''])

            # query the updated robots position in the environment and extract the information with the function robot_location
            self.query_position=client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
            self.robot_position=self.robot_location(self.query_position.queried_objects)

            # updated the robots position in the environment with the desired location
            client.call('REPLACE','OBJECTPROP','IND',['isIn', 'Robot1', desPos, self.robot_position])
            client.call('REASON','','',[''])



            # Update the timing information of the robot's data property `now`
            self.rob_time = client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
            self.old_rob_time = self.time_set(self.rob_time.queried_objects)
            # compute the current time
            self.current_time=str(math.floor(time.time()))
            client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', self.current_time, self.old_rob_time])
            client.call('REASON','','',[''])
            
            # query the subclasses of the locations (ROOM, URGENT, CORRIDOR) from the ontology
            self.query_locations = client.call('QUERY','CLASS','IND',[desPos, 'true'])
            self.subclass_location = self.query_locations.queried_objects

            # For debugging uncomment the line below
            #print('The subclass of the location is: ', subclass_location)
            
            if self.subclass_location == ['URGENT'] or self.subclass_location == ['ROOM']:
                # Update the timing information of the room's data property `visitedAt`
                self.room_time = client.call('QUERY','DATAPROP','IND',['visitedAt', desPos])
                self.old_room_time = self.time_set(self.room_time.queried_objects)
                # compute the current time
                self.current_time=str(math.floor(time.time()))
                client.call('REPLACE','DATAPROP','IND',['visitedAt', desPos, 'Long', self.current_time, self.old_room_time])
                client.call('REASON','','',[''])


        # query the updated robots position in the environment and extract the information with the function robot_location
        self.query_position=client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
        self.robot_position=self.robot_location(self.query_position.queried_objects)
        self.go_to_coordinate(self.robot_position)

        # For debugging uncomment the line below
        #print('The robot is in: ', self.robot_position)

        client.call('REASON','','',[''])

        #return self.robot_position

        