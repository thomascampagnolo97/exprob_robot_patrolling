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
from std_msgs.msg import Bool

from armor_api.armor_client import ArmorClient

# Import constant name defined to structure the architecture
from exprob_robot_patrolling import architecture_name_mapper as anm

# Arguments for loading and create the ontology
rp = rospkg.RosPack()
assignment_path = rp.get_path('exprob_robot_patrolling')
ONTOLOGY_FILE_PATH = os.path.join(assignment_path, "topological_map", "topological_map.owl")
WORLD_ONTOLOGY_FILE_PATH = os.path.join(assignment_path, "topological_map", "world_surveillance.owl") # final map OWL, also used for debugging
WEB_PATH = 'http://bnc/exp-rob-lab/2022-23'


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

    rospy.init_node(anm.NODE_WORLD_GENERATOR, anonymous=True)
    
    pub = rospy.Publisher(anm.TOPIC_WORLD_LOAD, Bool, queue_size=10)
    pub.publish(False)

    print("Building the environment...")

    client = ArmorClient("example", "ontoRef")

    client.call('LOAD','FILE','',[ONTOLOGY_FILE_PATH, WEB_PATH, 'true', 'PELLET', 'false'])
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'E', 'D6'])
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'E', 'D7'])
    print('Doors D6 and D7 associated to room E (RECHARGING ROOM)')

    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'C1', 'D1'])
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'C1', 'D2'])
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'C1', 'D5'])
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'C1', 'D6'])
    print('Doors D1, D2, D5, D6 associated to corridor C1')

    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'C2', 'D3'])
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'C2', 'D4'])
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'C2', 'D5'])
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'C2', 'D7'])
    print('Doors D3, D4, D5, D7 associated to corridor C2')

    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'R1', 'D1'])
    print('Door D1 associated to room R1')

    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'R2', 'D2'])
    print('Door D2 associated to room R2')

    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'R3', 'D3'])
    print('Door D3 associated to room R3')

    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'R4', 'D4'])
    print('Door D4 associated to room R4')

    # declaration of disjunction of all instances of the Abox
    client.call('DISJOINT','IND','',['R1','R2','R3','R4','E','C1','C2','D1','D2','D3','D4','D5','D6','D7'])

    # definition of the init position of the robot
    client.call('ADD','OBJECTPROP','IND',['isIn', 'Robot1', anm.INIT_LOCATION])
    print('Initialization: Robot1 is in: ', anm.INIT_LOCATION)
    client.call('REASON','','',[''])
    print('Reasoning...')

    rospy.sleep(2)

    # Start the timestamp in every location to retrieve when a location becomes urgent
    # Visit and timestamp for R1
    client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', 'R1', 'E'])
    client.call('REASON','','',[''])
    rob_time = client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
    old_rob_time = timestamp_computation(rob_time.queried_objects)
    print('R1 last visit time: ', old_rob_time)
    current_time=str(math.floor(time.time()))
    print('R1 current time: ', current_time)

    client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', current_time, old_rob_time])
    client.call('REASON','','',[''])
    
    client.call('ADD','DATAPROP','IND',['visitedAt','R1', 'Long', current_time])
    client.call('REASON','','',[''])

    rospy.sleep(1)

    # Visit and timestamp for R2
    client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', 'R2', 'R1'])
    client.call('REASON','','',[''])
    rob_time = client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
    old_rob_time = timestamp_computation(rob_time.queried_objects)
    print('R2 last visit time: ', old_rob_time)
    current_time=str(math.floor(time.time()))
    print('R2 current time: ', current_time)
    client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', current_time, old_rob_time])
    client.call('REASON','','',[''])

    client.call('ADD','DATAPROP','IND',['visitedAt','R2', 'Long', current_time])
    client.call('REASON','','',[''])

    rospy.sleep(1)

    # Visit and timestamp for R3
    client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', 'R3', 'R2']) 
    client.call('REASON','','',[''])
    rob_time = client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
    old_rob_time = timestamp_computation(rob_time.queried_objects)
    print('R3 last visit time: ', old_rob_time)
    current_time=str(math.floor(time.time()))
    print('R3 current time: ', current_time)    
    client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', current_time, old_rob_time])
    client.call('REASON','','',[''])

    client.call('ADD','DATAPROP','IND',['visitedAt','R3', 'Long', current_time])
    client.call('REASON','','',[''])

    rospy.sleep(1)

    # Visit and timestamp for R4
    client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', 'R4', 'R3'])  
    client.call('REASON','','',[''])
    rob_time = client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
    old_rob_time = timestamp_computation(rob_time.queried_objects)
    print('R4 last visit time: ', old_rob_time)
    current_time=str(math.floor(time.time()))
    print('R4 current time: ', current_time)
    client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', current_time, old_rob_time])
    client.call('REASON','','',[''])

    client.call('ADD','DATAPROP','IND',['visitedAt','R4', 'Long', current_time])
    client.call('REASON','','',[''])

    rospy.sleep(1)
    
    client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', anm.INIT_LOCATION, 'R4'])
    client.call('REASON','','',[''])

    rob_time = client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
    old_rob_time = timestamp_computation(rob_time.queried_objects)
    current_time=str(math.floor(time.time()))
    client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', current_time, old_rob_time])
    client.call('REASON','','',[''])

    # save the final ontology
    client.call('SAVE','','',[WORLD_ONTOLOGY_FILE_PATH])

    while not rospy.is_shutdown():
        pub.publish(True)
    

if __name__ == '__main__':

    try:
    	build_world()
    except rospy.ROSInterruptException:
    	pass
   