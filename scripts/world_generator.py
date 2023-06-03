#!/usr/bin/env python

"""
.. module:: world_generator
    :platform: Unix
    :synopsis: Python code to create the OWL ontology 

.. moduleauthor:: Thomas Campagnolo <s5343274@studenti.unige.it>

ROS Node to create the ontology map and initialize the timestamps for the rooms.
The 2D environment is composed by:
    - 4 rooms (R1, R2, R3, R4);
    - 2 corridors (C1, C2);
    - 1 special room (E), this is the charge location and also the init location;
    - 7 doors (D1, D2, D3, D4, D5, D6, D7).
   
Publishes to:
    /world_loading: a boolean flag to communicate when the environment is created
"""


# An example of the topological map can be the follow:
#   _________________________________
# 	|               				|
# 	|		        E               |
# 	|				                |
# 	|__________D6_______D7__________|
# 	|	    |   	|   	|	    |
# 	|  R1	|  C1	|  C2	|   R3	|
# 	|	    |	    |	    |	    |
# 	|	    D1	    |	    D3	    |
# 	|	    |	    |	    |	    |
# 	|_______|	    |	    |_______|
# 	|	    |      D5	    |	    |
# 	|	    |	    |	    |	    |
# 	|	    D2	    |	    D4	    |
# 	|	    |	    |	    |	    |
# 	|  R2	|       |	    |   R4	|
# 	|_______|_______|_______|_______|


import rospy
import rospkg
import os
import sys
import time
import math
import random
from std_msgs.msg import Bool, String

from armor_api.armor_client import ArmorClient

# Import constant name defined to structure the architecture
from exprob_robot_patrolling import architecture_name_mapper as anm

from exprob_robot_patrolling.srv import RoomInformation

markers_list = []

# Arguments for loading and create the ontology
rp = rospkg.RosPack()
assignment_path = rp.get_path('exprob_robot_patrolling')
ONTOLOGY_FILE_PATH = os.path.join(assignment_path, "topological_map", "topological_map.owl")
WORLD_ONTOLOGY_FILE_PATH = os.path.join(assignment_path, "topological_map", "world_surveillance.owl") # final map OWL, also used for debugging
WEB_PATH = 'http://bnc/exp-rob-lab/2022-23'



def extract_aruco(string):
    """
    Call back function that extracts Aruco marker IDs from a string and appends them to the global list of markers if they are not 
    already present.

    Args:
        - String 
    """

    global markers_list
    for word in string.data.split():
        if word.isdigit():
            num = int(word)
            if 10 < num < 18 and num not in markers_list:
                markers_list.append(num)
                print("Marker list: ", markers_list)
    
    return markers_list


def timestamp_computation(list):
    """
    Function to clean the queried time stamp for both Rooms and Robot's data property.

    Args:
        list: the list of queried objects section of the Armor service message.

    Returns:
        timestamp: elements of time information
    
    """

    timestamp = ''

    for i in list:
        for element in range(1, 11):
            timestamp=timestamp+i[element]
     
    return timestamp


def build_world():
    """
    This main function initialize the :mod:`world_generator` node, the publisher and allows to use the 
    `Armor commands <https://github.com/EmaroLab/armor/blob/master/commands.md>`_ to create the final ontology.
    It will publish a boolean that will be passed to the state ``BUILD_WORLD`` of the FSM, advertised by :mod:`fsm_behaviour`.

    """

    global markers_list

    print("Building the environment with the marker list: ", markers_list)

    client = ArmorClient("example", "ontoRef")
    client.call('LOAD','FILE','',[ONTOLOGY_FILE_PATH, WEB_PATH, 'true', 'PELLET', 'false'])

    rospy.wait_for_service('/room_info')
    roomInfo_srv = rospy.ServiceProxy('/room_info', RoomInformation)

    individuals = []

    for markers in markers_list: 
        res = roomInfo_srv(markers)
        roomID =res.room
        individuals.append(roomID)
        room_X = res.x
        room_Y = res.y

        print("Individials before: ", individuals)

        client.manipulation.add_dataprop_to_ind("X_point", roomID , "float", str(room_X))
        client.manipulation.add_dataprop_to_ind("Y_point", roomID , "float", str(room_Y))

        for c in res.connections:
            c.through_door
            individuals.append(c.through_door)
            client.call('ADD','OBJECTPROP','IND',['hasDoor', roomID, c.through_door])
            print("Individials after: ", individuals)
            print("c: ", c)

    
    #set() method is used to convert any iterable to sequence of distinct elements
    client.call('DISJOINT','IND','', list(set(individuals)))
    client.call('REASON','','',[''])
    
    # Initializing the rooms visited_at dataproperty
    for i in list(set(individuals)):
        if i.startswith('R'):
            client.manipulation.add_dataprop_to_ind("visitedAt", i, "Long", str(math.floor(time.time())))
            rospy.sleep(random.uniform(0.0, 1.5))
    client.call('REASON','','',[''])

    #Strat drom room E
    client.call('ADD', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', anm.INIT_LOCATION])
    client.call('REASON','','',[''])
    
    #Update time property

    rob_time = client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
    old_rob_time = timestamp_computation(rob_time.queried_objects)
    current_time=str(math.floor(time.time()))
    client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', current_time, old_rob_time])
   
    client.call('REASON','','',[''])

    # save the final ontology
    client.call('SAVE','','',[WORLD_ONTOLOGY_FILE_PATH])    

    


def marker_reader():
    """
    ROS Node to read the data from the subscribed topic '/marker_publisher/target' and extract the markers' ids. 
    When all the markers are detected, the node will shutdown the marker_publisher node, publish a boolean flag to the '/loader' topic 
    and call the function :mod:load_std_map.

    Subscribes to:
        - '/marker_publisher/target' a string containing the detected markers' ids.

    Publishes to:
        - '/loader' a boolean flag to communicate when the map is ready to be built.

    """

    rospy.init_node(anm.NODE_WORLD_GENERATOR, anonymous=True)
    
    pub = rospy.Publisher(anm.TOPIC_WORLD_LOAD, Bool, queue_size=10)
    pub.publish(False)
    sub = rospy.Subscriber("/marker_publisher/target", String, extract_aruco)

    while not rospy.is_shutdown():
        if len(markers_list)>=7:
            print("ALL MARKERS DETECTED")
            sub.unregister()
            build_world()
            pub.publish(True)
            rospy.sleep(3)
            rospy.signal_shutdown(anm.NODE_WORLD_GENERATOR)  

if __name__ == '__main__':

    try:
    	marker_reader()
    except rospy.ROSInterruptException:
    	pass
   