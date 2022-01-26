#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Robotiq, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotiq, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2012, Robotiq, Inc.
# Revision $Id$

"""@package docstring
Command-line interface for sending simple commands to a ROS node controlling a 2F gripper.

This serves as an example for publishing messages on the 'Robotiq2FGripperRobotOutput' topic using the 'Robotiq2FGripper_robot_output' msg type for sending commands to a 2F gripper.
"""

import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
import rospy
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input  as inputMsg
import time
from alive_progress import alive_bar
import cv2
import zmq


# current position of the fingers of the gripper
current_position = 0


def close_gripper(value):

    """
    Closes the gripper from current_position to current_position + value

    -Arguments:
        -value: Int that represents the value we want to close from current_position.
    
    -Returns: An object representing the msg of the topic called Robotiq2FGripperOutput
    """

    # current_position has to be a global variable
    global current_position

    # reading output values to extract current position
    command = outputMsg.Robotiq2FGripper_robot_output()
    command.rACT = 1
    command.rGTO = 1
    command.rSP  = 0
    command.rFR  = 0
    # add the value to close the gripper
    if value <= 255:
        command.rPR = value
    else:
        command.rPR = 255
    print("Close: ", command.rPR)

    return command


def open_gripper(value):

    """
    Opens the gripper from current_position to current_position - value

    -Arguments:
        -value: Int that represents the value we want to open from current_position.
    
    -Returns: An object representing the msg of the topic called Robotiq2FGripperOutput
    """

    global current_position

    command = outputMsg.Robotiq2FGripper_robot_output()
    command.rACT = 1
    command.rGTO = 1
    command.rSP  = 0
    command.rFR  = 0
    # substract the value to open the gripper
    value = abs(value)
    if (current_position - value) > 0:
        command.rPR = current_position - value
    else:
        command.rPR = 0
    print("Open: ", command.rPR)


    return command


def callback(msg):

    """
        Reads current position from requested output position. It's important to read
    the requested and not the current because the current is obtained from the encoders,
    and is not always the same as the requested.

        -Arguments:
            -msg: Ros msg of Robotiq2FGripperRobotInput topic.
        
    """

    global current_position
    current_position = msg.gPR
    #print(current_position)


def publish(pub, command):
    a = time.time()
    b = time.time()

    while (b-a) < 0.5:
        pub.publish(command)
        b = time.time()


def main():
    # use log_leve = rospy.DEBUG for debugging.
    # rospy.init_node("comm_test", log_level=rospy.DEBUG)
    rospy.init_node('Robotiq2FGripperDataset')
    
    pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output)

    sub = rospy.Subscriber('Robotiq2FGripperRobotInput', inputMsg.Robotiq2FGripper_robot_input, callback)

    rospy.loginfo("HEY! I'M USING ROS!")

    graspings = 3
    iters_per_grasping = 7
    time_per_iter = 4
    time_per_grasping = iters_per_grasping*time_per_iter # seconds
    time_between_graspings = 7
    max_grasping = [90, 100, 100]
    cycles = 0


    PORT = "5556"

    context = zmq.Context()
    client = context.socket(zmq.REQ)

    client.connect("tcp://localhost:%s" % PORT)
    

    for grasp in range(graspings):

        print("GRASPING {}".format(grasp))

        a = time.time()
        b = time.time()

        # we consider an iteration the process of closing, holding and opening the gripper
        with alive_bar(iters_per_grasping, title="RECORDING!!") as pbar_one_grasp:

            while (b-a) < time_per_grasping: # each iteration lasts 4 seconds
                
                if rospy.is_shutdown():
                    print('shutdown')
                    break

                #c = time.time()
                # close
                print("closing...")
                command = close_gripper(max_grasping[grasp])  
                publish(pub, command)
                client.send_pyobj(1)    
                data = client.recv_pyobj()
                rospy.sleep(0.5)

                # hold
                print("holding...")
                client.send_pyobj(0) 
                data = client.recv_pyobj()
                rospy.sleep(1)

                # open
                print("opening...")
                client.send_pyobj(1)    
                data = client.recv_pyobj()
                command = open_gripper(50)
                publish(pub, command)
                rospy.sleep(0.5)
                
                # no contact
                client.send_pyobj(0)    
                data = client.recv_pyobj()
                rospy.sleep(1)

                b = time.time()

                pbar_one_grasp()
                #print(b-c)
                cycles += 1
                
        print("Cycles: {}".format(cycles))

        if grasp < graspings-1:
            command = open_gripper(255)
            publish(pub, command)
            with alive_bar(5, title="CHANGE OF GRASPING!!") as pbar_change_grasping:
                for i in range(5):
                    time.sleep(1)
                    pbar_change_grasping()
            

    command = open_gripper(255)
    publish(pub, command)
    client.send_pyobj(5)



if __name__ == '__main__':
    main()
