# Crossing Realities: Connecting the Virtual and the Physical World for Remote Inspections

Daphne Barretto, Manuel Kreutle, Alex Glaser

*[INMM & ESARDA 2023 Joint Annual Meeting Proceedings Paper.](https://resources.inmm.org/annual-meeting-proceedings/crossing-realities-connecting-virtual-and-physical-world-remote)*

*[INMM & ESARDA 2023 Joint Annual Meeting Proceedings Slides.](https://sgs.princeton.edu/sites/default/files/2023-05/INMM-VR-2023-REV1.pdf)*

*Abstract.* Nuclear disarmament is the only solution to the threat of a global nuclear war. Traditional verification approaches of nuclear arms-control agreements towards disarmament have placed a strong emphasis on onsite inspections. Considering the large numbers of U.S. and Russian warheads as well as situations of political tensions or restricted global travel, in-person onsite inspections require a significant amount of time, resources, and trust between parties. As an alternative, remote inspections could facilitate and accelerate the verification process without the same requirements. While prior work has discussed how virtual reality could potentially be used for training and capacity-building in nuclear verification, we extend this idea by using virtual reality to combine virtual and in-person activities into remote nuclear verification inspections that reduce the intrusiveness and cost of inspections. Our approach utilizes bidirectional data flows and interaction between the inspector’s virtual environment and the host’s physical nuclear facility. We implement a full virtual reality environment and a physical demonstration to explore the technology and challenges of remote inspections conducted in virtual reality. For one direction of data flow, we use fiducial markers to estimate the position and orientation of key inspection elements in the physical world (e.g., warhead, robot, room) via an RGB camera, provide the estimated position and orientation data to a virtual reality project in Unreal Engine 5, and use 3D models of the tracked objects to show their relative positions and orientations in an immersive virtual reality environment with real-time updates. This provides inspectors in virtual reality with the key information from the physical world without revealing sensitive information. For the other direction of data flow, we use a mobile robot that can be controlled from the virtual reality environment so that it moves in the physical world. The robot acts as a placeholder for objects relevant to interact with during inspections (e.g., a radiation detector) and provides inspectors with a way to interact with the physical world even though they are not present. Combined, this work demonstrates and discusses how virtual reality could be used for remote nuclear verification inspections.

# Technology Setup

This work utilizes ArUCO Markers for pose estimation, an iRobot Create 3 and its corresponding SDK, and Unreal Engine 5 for virtual reality development. To set up a conda environment with the dependency requirements for this project, use ```conda create --name <env> --file requirements.txt```.

# Demo

In ```aruco_markers_pose_estimation```, run ```python3 pose_estimation.py --K_Matrix calibration_matrix.npy --D_Coeff distortion_coefficients.npy --type DICT_4X4_100``` to run the pose estimation code. This code calculates the pose estimation for the robot (ids 0-5), the warhead (ids 6-11), and the room (ids 12-17). The pose estimation is placed into ```robot_pose.txt```, ```warhead_pose.txt```, and ```room_pose.txt``` based on the three filepaths in ```pose_estimation.py```.

Run ```move_robot.py``` to make the iRobot Create 3 move according to the joystick values file. The joystick values file written from Unreal Engine 5 and read by ```move_robot.py```, which specifies the filepath to read from. This file can be updated automatically by running the Unreal Engine 5 project simultaneously.

Open the Unreal Engine 5 project, and run using a virtual reality headset. The left joystick is used to send robot commands for turning either direction or moving forward. The player can move around the virtual reality representation of the disarmament location to remotely inspect the environment.

# Acknowledgements

This work utilizes the [ArUCo-Markers-Pose-Estimation-Generation-Python](https://github.com/GSNCodes/ArUCo-Markers-Pose-Estimation-Generation-Python) for pose estimation with ArUCO Markers and [the Python SDK for iRobot Edu robots](https://github.com/iRobotEducation/irobot-edu-python-sdk).
