#!/usr/bin/env python

import rospy
import rospkg
import os
import sys
import yaml
import random
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

yamlpath = 'lab2_data.yaml'

if __name__ == '__main__':

    # Initialize rospack
    rospack = rospkg.RosPack()
    # Get path to yaml
    lab2_path = rospack.get_path('lab2pkg_py')
    yamlpath = os.path.join(lab2_path, 'scripts', 'lab2_data.yaml')

    with open(yamlpath, 'r') as f:
        try:
            # Load the data as a dict
            data = yaml.load(f)
            # Load block position
            block_xy_pos = data['block_xy_pos']
           
        except:
            sys.exit()

    # Initialize ROS node
    rospy.init_node('ur3_gazebo_spawner', anonymous=True)
    # Initialize ROS pack
    rospack = rospkg.RosPack()
    # Get path to block
    ur_path = rospack.get_path('ur_description')
    block_path = os.path.join(ur_path, 'urdf', 'block.urdf')
    block_red_path = os.path.join(ur_path, 'urdf', 'block_red.urdf')
    block_green_path = os.path.join(ur_path, 'urdf', 'block_green.urdf')
    bowl_path = os.path.join(ur_path, 'urdf', 'geo_bowl.urdf')
    # Wait for service to start
    rospy.wait_for_service('gazebo/spawn_urdf_model')
    spawn = rospy.ServiceProxy('gazebo/spawn_urdf_model', SpawnModel)

    # 0-indexed
    starting_location = 0
    
    pose = Pose(Point(0.2, 0.5, 0), Quaternion(0, 0, 0, 0))
    spawn('bowl1', open(bowl_path, 'r').read(), 'bowl1', pose, 'world')
    pose = Pose(Point(0.2, 0.3, 0), Quaternion(0, 0, 0, 0))
    spawn('bowl2', open(bowl_path, 'r').read(), 'bowl2', pose, 'world')
