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
#first_image_number = 0



def setup_digit(idx):

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

    idx50 = "D00050"
    idx55 = "D00055"

    digit50 = setup_digit(idx50)
    digit55 = setup_digit(idx55)
    
    cont = first_image_number
  
    path_save = ["/home/aurova/Desktop/julio_manipulacion_digit/dataset/" + str(args.object) + "/" + "estable/",
                 "/home/aurova/Desktop/julio_manipulacion_digit/dataset/" + str(args.object) + "/" + "inestable/"]

    port, server = init_comms("5556")

    data = None
    exit_program = True

    while(exit_program):

        try:
            print("Receiving...")
            data = server.recv_pyobj(flags=zmq.NOBLOCK) # waits 
        except zmq.ZMQError as e:
            if e.errno == zmq.EAGAIN:
                pass
            else:
                traceback.print_exc()

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

            print("SAVING...")
            cv2.imwrite(path_save[data] + "50_" + str(cont) + ".jpg", frame50)
            cv2.imwrite(path_save[data] + "55_" + str(cont) + ".jpg", frame55)
            cont += 1

            try:
                print("Sending...")
                server.send_pyobj(1, protocol=0)
            except zmq.ZMQError as e:
                if e.errno == zmq.EAGAIN:
                    pass
                else:
                    traceback.print_exc()

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        elif data == 5:
            exit_program = False

        else:
            continue
