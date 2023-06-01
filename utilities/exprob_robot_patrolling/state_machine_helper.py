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
import time
import math
import random
import smach
import smach_ros

from armor_api.armor_client import ArmorClient
from std_msgs.msg import Bool



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

    def motion_control(self, robPos, desPos):
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
        
        # query the ontology about the room's object property `connectedTo` (spatial information), for the actual robot position and the desided one
        self.robot_connections = client.call('QUERY','OBJECTPROP','IND',['connectedTo', robPos])
        self.arrival_connections = client.call('QUERY','OBJECTPROP','IND',['connectedTo', desPos])
        
        # extract from the queries the robot possible moves (spatial information)
        self.robot_possible_moves = self.rooms_search(self.robot_connections.queried_objects)
        self.arrival_moves = self.rooms_search(self.arrival_connections.queried_objects)

        # For debugging uncomment the two lines below
        #print('Actual robot position, possible moves: ', robot_possible_moves)
        #print('Desired robot position, possible moves: ', arrival_moves)

        # Find a possible path in the shared location between the possible moves of the actual and desired positions
        self.shared_location = self.path_planning(self.robot_possible_moves, self.arrival_moves, robPos)

        # For debugging uncomment the line below
        #print('The shared location is: ', shared_location)
        
        # query the subclasses of the locations (ROOM, URGENT, CORRIDOR) from the ontology
        self.query_locations = client.call('QUERY','CLASS','IND',[desPos, 'true'])
        self.subclass_location = self.query_locations.queried_objects

        # For debugging uncomment the line below
        #print('The subclass of the location is: ', subclass_location)
        
        if self.subclass_location == ['URGENT'] or self.subclass_location == ['ROOM']:
            if self.shared_location == '':
                # if there are not shared location between the actual robot position and the desired one in the map, 
                # e.g. the robot is in R2 and R3 becomes urgent
                if 'C1' in self.robot_possible_moves:
                    # replace the robot in the corridor C1 if C1 is in the robot possible moves list,
                    # e.g. if the robot is in R2, C1 is a element of the robot possible moves list
                    client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', robPos, 'C1'])
                    client.call('REASON','','',[''])
                elif 'C2' in self.robot_possible_moves:
                    # replace the robot in the corridor C2 if C2 is in the robot possible moves list,
                    # e.g. if the robot is in R3, C2 is a element of the robot possible moves list
                    client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', robPos, 'C2'])
                    client.call('REASON','','',[''])
                
                # query the ontology about the position of the robot in the environment and extract the information with the function robot_location
                self.query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
                self.robot_position = self.robot_location(self.query_position.queried_objects)

                #print('Robot position updated: ', robot_position) # For debugging uncomment this line
                
                # query the ontology about the room's object property `connectedTo` (spatial information), update the information about the possible moves
                self.robot_connections = client.call('QUERY','OBJECTPROP','IND',['connectedTo', self.robot_position])
                self.robot_possible_moves = self.rooms_search(self.robot_connections.queried_objects)
                # update the path of the shared location between the possible moves of the actual and desired positions
                self.shared_location = self.path_planning(self.robot_possible_moves, self.arrival_moves, self.robot_position)
                
                # print('Shared locations updated: ', shared_location) # For debugging uncomment this line

                # replace the robot position with the shared location
                client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', self.shared_location, robPos])
                client.call('REASON','','',[''])
                # replace the shared position with the desired one
                client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', desPos, self.shared_location])
                client.call('REASON','','',[''])
                
                # query the ontology about the position of the robot in the environment and extract the information with the function robot_location
                self.query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
                self.robot_position = self.robot_location(self.query_position.queried_objects)
                
                #print('New robot position updated: ', robot_position) # For debugging uncomment this line
                
                rospy.sleep(self.wait_time)

                # updating all the timing information of the robot's and room's data properties `now` and `visitedAt` respectively
                # `now` data property
                self.rob_time = client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
                self.old_rob_time = self.time_set(self.rob_time.queried_objects)
                # compute the current time
                self.current_time=str(math.floor(time.time()))
                client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', self.current_time, self.old_rob_time])
                client.call('REASON','','',[''])
                # `visitedAt` data property
                self.room_time = client.call('QUERY','DATAPROP','IND',['visitedAt', desPos])
                self.old_room_time = self.time_set(self.room_time.queried_objects)
                client.call('REPLACE','DATAPROP','IND',['visitedAt', desPos, 'Long', self.current_time, self.old_room_time])
                client.call('REASON','','',[''])
            
            elif self.shared_location == robPos:
                # if a shared location is the actual robot position
                # replace the shared position with the desired one
                client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', desPos, self.shared_location])
                client.call('REASON','','',[''])
                # update the robot position in the ontology and extract the information
                self.query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
                self.robot_position = self.robot_location(self.query_position.queried_objects)

                rospy.sleep(self.wait_time)

                # updating all the timing information of the robot's and room's data properties `now` and `visitedAt` respectively
                # `now` data property
                self.rob_time = client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
                self.old_rob_time = self.time_set(self.rob_time.queried_objects)
                self.current_time=str(math.floor(time.time()))
                client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', self.current_time, self.old_rob_time])
                client.call('REASON','','',[''])
                # `visitedAt` data property
                self.room_time = client.call('QUERY','DATAPROP','IND',['visitedAt', desPos])
                self.old_room_time = self.time_set(self.room_time.queried_objects)
                client.call('REPLACE','DATAPROP','IND',['visitedAt', desPos, 'Long', self.current_time, self.old_room_time])
                client.call('REASON','','',[''])

            else:
                # if there is a location that is shared between the two list (see `path_planning` function) 
                # replace the robot position with the shared location
                client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', self.shared_location, robPos])
                client.call('REASON','','',[''])
                # replace the shared position with the desired one
                client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', desPos, self.shared_location])
                client.call('REASON','','',[''])
                # update the robot position in the ontology and extract the information
                self.query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
                self.robot_position = self.robot_location(self.query_position.queried_objects)

                rospy.sleep(self.wait_time)

                # updating all the timing information of the robot's and room's data properties `now` and `visitedAt` respectively
                # `now` data property
                self.rob_time = client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
                self.old_rob_time = self.time_set(self.rob_time.queried_objects)
                # compute the current time
                self.current_time=str(math.floor(time.time()))
                client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', self.current_time, self.old_rob_time])
                client.call('REASON','','',[''])
                # `visitedAt` data property
                self.room_time = client.call('QUERY','DATAPROP','IND',['visitedAt', desPos])
                self.old_room_time = self.time_set(self.room_time.queried_objects)
                client.call('REPLACE','DATAPROP','IND',['visitedAt', desPos, 'Long', self.current_time, self.old_room_time])
                client.call('REASON','','',[''])
            
            return self.robot_position
        
        else:
            # if the desired position is not a URGENT subclass but is a CORRIDOR or the special room E
            if self.shared_location == '':
                # if there are not shared location between the actual robot position and the desired one in the map 
                # e.g. the robot is in R2 and the desired location is C2
                if 'C1' in self.robot_possible_moves:
                    # replace the robot in the corridor C1 if C1 is in the robot possible moves list,
                    # e.g. if the robot is in R2, C1 is a element of the robot possible moves list
                    client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', robPos, 'C1'])
                    client.call('REASON','','',[''])
                elif 'C2' in self.robot_possible_moves:
                    # replace the robot in the corridor C2 if C2 is in the robot possible moves list,
                    # e.g. if the robot is in R3, C2 is a element of the robot possible moves list
                    client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', robPos, 'C2'])
                    client.call('REASON','','',[''])
                
                # update the robot position in the ontology and extract the information
                self.query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
                self.robot_position = self.robot_location(self.query_position.queried_objects)
                # replace the desired position with the actual updated
                client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', desPos, self.robot_position])
                client.call('REASON','','',[''])
                # the final query is needed again to return the updated result
                self.query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
                self.robot_position = self.robot_location(self.query_position.queried_objects)
            else:
                # if there is a shared location between the actual robot position and the desired one in the map 
                client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', self.shared_location, robPos])
                client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', desPos, self.shared_location])
                client.call('REASON','','',[''])
                # final query to return the updated robot's position
                self.query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
                self.robot_position = self.robot_location(self.query_position.queried_objects)

            return self.robot_position    
        