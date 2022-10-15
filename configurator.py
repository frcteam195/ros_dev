#!/usr/bin/env python3

#Scan All folders in workspace root
#Identify all _node folders and _Robot folder
#Allow adding ROS and third party depends to _Robot
#Allow adding libraries and nodes to _node CMAKE files
#Create new nodes?

import os

script_dir = os.path.abspath( os.path.dirname( __file__ ) )
 
rootdir = os.path.normpath(script_dir + '/../')
nodes = []
robot_folder = ""
for file in os.listdir(rootdir):
    d = os.path.join(rootdir, file)
    if os.path.isdir(d):
        if d.endswith("_node"):
            nodes.append(d)
        if d.endswith("_Robot"):
            robot_folder = d

print(nodes)
print(robot_folder)