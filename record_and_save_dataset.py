# Copyright (c) Facebook, Inc. and its affiliates. All rights reserved.

# This source code is licensed under the license found in the LICENSE file in the root directory of this source tree.

import logging
import pprint
import time
import cv2
from digit_interface.digit import Digit
from digit_interface.digit_handler import DigitHandler
import argparse
import zmq
import traceback
import sys


parser = argparse.ArgumentParser()

parser.add_argument('-i', '--init', type=str, help='First image number to save')
parser.add_argument('-o', '--object', type=str, help='Image folder')

args = parser.parse_args()

first_image_number = int(args.init)


def setup_digit(idx):

    """
    Setups one digit sensor called idx.

    -Arguments:
        -idx: The ID of the sensor. It is a string like "D00050".
    
    -Returns: 
        An object representing the digit sensor. We will use it to 
        get images from the sensor.

    """

    logging.basicConfig(level=logging.DEBUG)

    # Print a list of connected DIGIT's
    digits = DigitHandler.list_digits()
    print("Connected DIGIT's to Host:")
    pprint.pprint(digits)

    # Connect to a Digit device with serial number with friendly name
    digit = Digit(idx, "Left Gripper")
    digit_cap = DigitHandler.find_digit(idx)
    digit.connect()

    # Print device info
    print(digit.info())

    # Change DIGIT resolution to QVGA
    qvga_res = DigitHandler.STREAMS["QVGA"]
    digit.set_resolution(qvga_res)

    # Change DIGIT FPS to 15fps
    fps_30 = DigitHandler.STREAMS["QVGA"]["fps"]["30fps"]
    digit.set_fps(fps_30)

    return digit


def init_comms(str_port):
    """
        Initialises communications with robotiq gripper.

        -Arguments:
            -str_port: String representing the number of the port.
        
        Returns: The number of the port (string) and the object of the server that
        allows us to send and read msgs.
    """

    port = str_port
    context_socket = zmq.Context()
    server = context_socket.socket(zmq.REP)
    server.bind("tcp://*:%s" % port)

    return port, server


if __name__ == '__main__':

    # define digit's idxs
    idx50 = "D00050"
    idx55 = "D00055"

    # digit's setup
    digit50 = setup_digit(idx50)
    digit55 = setup_digit(idx55)
    
    # cont is the first number of the first image for both sensors
    cont = first_image_number
  
    # a list where we save the paths where we want to save the images.
    path_save = ["/home/aurova/Desktop/julio_manipulacion_digit/dataset/" + str(args.object) + "/" + "estable/",
                 "/home/aurova/Desktop/julio_manipulacion_digit/dataset/" + str(args.object) + "/" + "inestable/"]

    # start comms with robotiq script
    port, server = init_comms("5556")

    # variables
    data = None
    exit_program = True

    # exit program will be true when robotiq script sends a 5 integer
    while(exit_program):

        # read in asynchronous mode
        try:
            print("Receiving...")
            data = server.recv_pyobj(flags=zmq.NOBLOCK)  
        except zmq.ZMQError as e:
            if e.errno == zmq.EAGAIN:
                pass
            else:
                traceback.print_exc()

        # when data is ready and is not exit program value, we start extracting
        # images and saving.
        if data is not None and data != 5:

            # Grab single frame from DIGIT
            frame50 = digit50.get_frame()
            frame55 = digit55.get_frame()

            #this is to wait for the colors to be in the same intensity.
            #if cont_start < 40:
            #    cont_start += 1
            #    continue

            cv2.imshow(idx50, frame50)
            cv2.imshow(idx55, frame55)

            # robotiq script sends number 0 to save in first path, 1 in the second.
            print("SAVING...")
            cv2.imwrite(path_save[data] + "50_" + str(cont) + ".jpg", frame50)
            cv2.imwrite(path_save[data] + "55_" + str(cont) + ".jpg", frame55)
            cont += 1

            # we have to send in asynchronous mode too, because the cycle has to be
            # send-receive. We can't send or receive twice in a row.
            try:
                print("Sending...")
                server.send_pyobj(1, protocol=0)
            except zmq.ZMQError as e:
                if e.errno == zmq.EAGAIN:
                    pass
                else:
                    traceback.print_exc()

            # exit programm before finishing.
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # exit program
        elif data == 5:
            exit_program = False

        # next iteration
        else:
            continue
