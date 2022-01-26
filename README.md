# Automatic-Dataset-Recording
This repository contains the code that I usually use to create datasets using visual-based tactile sensors and a Robotiq 2f gripper.

First, I'll explain both scripts a little bit, and then I'll describe how the communications work and execution process.

# Record and save script (first script)
In this script, I get images from visual-based sensors and these images are saved in to disk. In this case, closing and opening (gripper movement) images are saved in one path, and holding and no contact images in another. But this depends on the task. 

# Robotiq2FGripperDataset (second script)
First of all, I use a 2 finger robotiq gripper in ros, so if you want to run this script, you'll have to create a caktin workspace with the robotiq package.

This script runs a main loop of graspings. A grasping is the pose I choose to grasp the object with the gripper and sensors. In each grasping, the program runs what I called an iteration: the process of closing, holding and opening the gripper. I can define how many iterations per grasping I want to do based on the time that one iteration lasts.

So, basically, the gripper is closing, holding and opening in contact with the object at the same time the first script is recording images. Thus, I can save tactile images automatically, I only have to hold the object in hand and change the pose.

# Communications
In my case, my sensors work with python 3 and the robotiq gripper works with python 2. Then, I can't use ros topics to communicate between them. However, it is really easy to set up a client-server communication process using zmq library in python.

The recording script works as the server, it is running in first place. Because I am using virtual envs, I have to run this command before running the recording script:  export OPENBLAS_CORETYPE=ARMV8 (running in jetson, maybe that's the reason I need this command).
Afterthat, I run python record_and_save_dataset.py -i ... -o ... -i indicates the name (number) of the first image for both sensors, and -o indicates the object (1,2,3,...).

When the first script is running, I can execute the second one (rosrun package script). Then, the whole process starts and everything would work fine!
