#!/usr/bin/env python

'''
We get inspirations of Tower of Hanoi algorithm from the website below.
This is also on the lab manual.
Source: https://www.cut-the-knot.org/recurrence/hanoi.shtml
'''

import os
import argparse
import copy
import time
import rospy
import rospkg
import numpy as np
import yaml
import sys
from lab2_header import *
from blob_search import *
from lab2_func import *
from math import pi

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = np.radians([120, -90, 90, -90, -90, 0])

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0

suction_on = True
suction_off = False

current_io_0 = False
current_position_set = False

Q = None
go_away = [270*pi/180.0, -90*pi/180.0, 90*pi/180.0, -90*pi/180.0, -90*pi/180.0, 135*pi/180.0]
z_height = 0.037
yaw = 90
P = []
red_bin = [0.2, 0.3]
green_bin = [0.2, 0.5]


"""
Whenever ur3/gripper_input publishes info this callback function is called.
"""
def input_callback(msg):

    global digital_in_0
    digital_in_0 = msg.DIGIN
    digital_in_0 = digital_in_0 & 1 # Only look at least significant bit, meaning index 0

"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

    global thetas
    global current_position
    global current_position_set

    thetas[0] = msg.position[0]
    thetas[1] = msg.position[1]
    thetas[2] = msg.position[2]
    thetas[3] = msg.position[3]
    thetas[4] = msg.position[4]
    thetas[5] = msg.position[5]

    current_position[0] = thetas[0]
    current_position[1] = thetas[1]
    current_position[2] = thetas[2]
    current_position[3] = thetas[3]
    current_position[4] = thetas[4]
    current_position[5] = thetas[5]

    current_position_set = True


def gripper(pub_cmd, loop_rate, io_0):

    global SPIN_RATE
    global thetas
    global current_io_0
    global current_position

    error = 0
    spin_count = 0
    at_goal = 0

    current_io_0 = io_0

    driver_msg = command()
    driver_msg.destination = current_position
    driver_msg.v = 1.0
    driver_msg.a = 1.0
    driver_msg.io_0 = io_0
    pub_cmd.publish(driver_msg)

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


def move_arm(pub_cmd, loop_rate, dest, vel, accel):

    global thetas
    global SPIN_RATE

    error = 0
    spin_count = 0
    at_goal = 0

    driver_msg = command()
    driver_msg.destination = dest
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1
            #rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


def move_block(pub_cmd, loop_rate, start_xw_yw_zw, target_xw_yw_zw, vel, accel):

    error = 0

    move_arm(pub_cmd, loop_rate, go_away, vel, accel)
    time.sleep(0.3)
    move_arm(pub_cmd, loop_rate, start_xw_yw_zw, vel, accel)
    time.sleep(0.5)
    gripper(pub_cmd, loop_rate, suction_on)
    time.sleep(0.3)

    if digital_in_0 == 0:
        error = 1
        move_arm(pub_cmd, loop_rate, go_away, 4.0, 4.0)
        gripper(pub_cmd, loop_rate, suction_off)
        time.sleep(0.3)
        print("No block found! Moving to the next position")
        return error

    move_arm(pub_cmd, loop_rate, go_away, vel, accel)
    time.sleep(0.3)

    move_arm(pub_cmd, loop_rate, target_xw_yw_zw, vel, accel)
    time.sleep(0.5)
    gripper(pub_cmd, loop_rate, suction_off)
    time.sleep(0.3)
    move_arm(pub_cmd, loop_rate, go_away, vel, accel)
    time.sleep(0.3)

    return error


class ImageConverter:

    def __init__(self, SPIN_RATE):

        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/image_converter/output_video", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/cv_camera_node/image_raw", Image, self.image_callback)
        self.image_sub = rospy.Subscriber("/cv_camera_node/camera_info", CameraInfo, self.camera_info_callback)

        self.loop_rate = rospy.Rate(SPIN_RATE)

        # Check if ROS is ready for operation
        while(rospy.is_shutdown()):
            print("ROS is shutdown!")

    def camera_info_callback(self, data):
        global P

        img_height = data.height
        img_width = data.width

        P = data.P
        #print(data)
        

    def image_callback(self, data):

        global xw_yw_G # store found green blocks in this list
        global xw_yw_R # store found yellow blocks in this list
        global P

        try:
          # Convert ROS image to OpenCV image
            raw_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv_image = cv2.flip(raw_image, -1)
        #cv2.line(cv_image, (0,50), (640,50), (0,0,0), 5)

        # You will need to call blob_search() function to find centers of green blocks
        # and yellow blocks, and store the centers in xw_yw_G & xw_yw_Y respectively.

        # If no blocks are found for a particular color, you can return an empty list,
        # to xw_yw_G or xw_yw_Y.

        # Remember, xw_yw_G & xw_yw_Y are in global coordinates, which means you will
        # do coordinate transformation in the blob_search() function, namely, from
        # the image frame to the global world frame.

        xw_yw_R = blob_search(cv_image, "red", P)
        xw_yw_G = blob_search(cv_image, "green", P)

def main():

    global home
    global Q
    global SPIN_RATE
    global z_height
    global yaw
    global go_away
    global xw_yw_G
    global xw_yw_R
    global red_bin
    global green_bin

    # Parser
    parser = argparse.ArgumentParser(description='Please specify if using simulator or real robot')
    parser.add_argument('--simulator', type=str, default='True')
    args = parser.parse_args()

    # Initialize rospack
    rospack = rospkg.RosPack()
    # Get path to yaml
    lab2_path = rospack.get_path('lab2pkg_py')
    yamlpath = os.path.join(lab2_path, 'scripts', 'lab2_data.yaml')

    with open(yamlpath, 'r') as f:
        try:
            # Load the data as a dict
            data = yaml.load(f)
            if args.simulator.lower() == 'true':
                Q = data['sim_pos']
            elif args.simulator.lower() == 'false':
                Q = data['real_pos']
            else:
                print("Invalid simulator argument, enter True or False")
                sys.exit()
            
        except:
            print("YAML not found")
            sys.exit()

    # Initialize ROS node
    rospy.init_node('lab2node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)
    sub_input = rospy.Subscriber('ur3/gripper_input', gripper_input, input_callback)

    dest = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    '''
    xWgrip = input("Please input xWgrip:")
    yWgrip = input("Please input yWgrip:")
    zWgrip = input("Please input zWgrip:")
    yaw_WgripDegree = input("Please input yaw_WgripDegree:")


    dest = lab_invk(float(xWgrip), float(yWgrip), float(zWgrip), float(yaw_WgripDegree))
	'''
    vel = 4.0
    accel = 4.0

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    # Initialize the rate to publish to ur3/command
    loop_rate = rospy.Rate(SPIN_RATE)

    move_arm(pub_command, loop_rate, go_away, vel, accel) 

    rospy.loginfo("move to go away")
    
    ic = ImageConverter(SPIN_RATE)
    time.sleep(5)

    print("xw_yw_R", xw_yw_R)
    print("xw_yw_G", xw_yw_G)

    for pos in xw_yw_R:
        print(pos)
        move_block(pub_command, loop_rate, lab_invk(pos[0], pos[1], z_height, yaw), lab_invk(red_bin[0], red_bin[1], z_height, yaw), vel, accel)

    for pos in xw_yw_G:
        print(pos)
        move_block(pub_command, loop_rate, lab_invk(pos[0], pos[1], z_height, yaw), lab_invk(green_bin[0], green_bin[1], z_height, yaw), vel, accel)

    sys.exit()


if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
