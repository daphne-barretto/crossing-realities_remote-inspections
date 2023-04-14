# pose_estimation.py, Daphne Barretto
# estimate the position of ArUCo markers specified for the robot and warhead, and
# write their ids, positions, angles, and timestamps to the appropriate files
# adapted from https://github.com/GSNCodes/ArUCo-Markers-Pose-Estimation-Generation-Python/blob/main/pose_estimation.py

'''
Sample Usage:-
python3 pose_estimation.py --K_Matrix calibration_matrix.npy --D_Coeff distortion_coefficients.npy --type DICT_4X4_100
'''

from datetime import datetime
from utils import ARUCO_DICT

import argparse
import cv2
import numpy as np
import sys
import time


def pose_estimation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):

    '''
    frame - Frame from the video stream
    matrix_coefficients - Intrinsic matrix of the calibrated camera
    distortion_coefficients - Distortion coefficients associated with your camera

    return:-
    frame - The frame with the axis drawn on it
    '''

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters_create()

    # detectMarkers()
    corners = []
    prev_corners = corners
    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=parameters)
    
    # try to read robot file
    robot_file_path = "C:\\Users\\PrincetonVR\\Documents\\Unreal Projects\\MyProject_VRTemplate_Robots\\Content\\Files\\robot_pose.txt"
    try:
        with open(robot_file_path, 'r') as file:
            robot_data = file.readlines()
    except PermissionError:
        return

    # try to read warhead file
    warhead_file_path = "C:\\Users\\PrincetonVR\\Documents\\Unreal Projects\\MyProject_VRTemplate_Robots\\Content\\Files\\warhead_pose.txt"
    try:
        with open(warhead_file_path, 'r') as file:
            warhead_data = file.readlines()
    except PermissionError:
        return
    
    # try to read room file
    room_file_path = "C:\\Users\\PrincetonVR\\Documents\\Unreal Projects\\MyProject_VRTemplate_Robots\\Content\\Files\\room_pose.txt"
    try:
        with open(room_file_path, 'r') as file:
            room_data = file.readlines()
    except PermissionError:
        return

    # If markers are detected
    if len(corners) > 0:
        for i in range(0, len(ids)):
            # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.168, matrix_coefficients,
                                                                       distortion_coefficients)

            # save previous rvec
            if len(prev_corners) > 0:
                prev_rvec, _, _ = cv2.aruco.estimatePoseSingleMarkers(prev_corners[i], 0.168, matrix_coefficients,
                                                                       distortion_coefficients)

            curr_id = ids[i][0]

            # convert rotation vector to euler angles
            # from https://github.com/mpatacchiola/deepgaze/issues/3#issuecomment-345405613
            rot_matrix = cv2.Rodrigues(rvec)[0]
            proj_matrix = np.hstack((rot_matrix, tvec[0][0].reshape((3,1))))
            euler_angles = cv2.decomposeProjectionMatrix(proj_matrix)[6]

            # convert previous rotation vector to euler angles
            if len(prev_corners) > 0:
                prev_rot_matrix = cv2.Rodrigues(prev_rvec)[0]
                prev_proj_matrix = np.hstack((prev_rot_matrix, tvec[0][0].reshape((3,1))))
                prev_euler_angles = cv2.decomposeProjectionMatrix(prev_proj_matrix)[6]
            else:
                prev_euler_angles = []

            # removing drastic "flickering", when markers appear to change rotation due to poor camera view of it
            flicker = False
            for i in range(len(prev_euler_angles)):
                diff = abs(prev_euler_angles[i] - euler_angles[i])
                if diff > 90 and diff < 270:
                    flicker = True
            if flicker:
                print("flicker")
                continue

            # format information to write
            str_to_write = str(curr_id)
            for element in tvec[0][0]:
                str_to_write = str_to_write + "," + str(element)
            for element in euler_angles:
                str_to_write = str_to_write + "," + str(element[0])
            curr_datetime = datetime.now()
            formatted_curr_datetime = "%04d.%02d.%02d-%02d.%02d.%02d" % (curr_datetime.year, curr_datetime.month, curr_datetime.day, curr_datetime.hour, curr_datetime.minute, curr_datetime.second)
            str_to_write += "," + formatted_curr_datetime + ";\n"
            
            # add information to write to the appropriate data
            if curr_id >= 0 and curr_id < 6:
                robot_data[curr_id] = str_to_write
            if curr_id >= 6 and curr_id < 12:
                warhead_data[curr_id - 6] = str_to_write
            if curr_id >= 12 and curr_id < 18:
                room_data[curr_id - 12] = str_to_write

            # Draw a square around the markers
            cv2.aruco.drawDetectedMarkers(frame, corners) 

            # Draw Axis
            cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)  

    print("robot_data")
    print(robot_data[0], end='\r')
    print(robot_data[1], end='\r')
    print(robot_data[2], end='\r')
    print(robot_data[3], end='\r')
    print(robot_data[4], end='\r')
    print(robot_data[5], end='\r')

    print("warhead_data")
    print(warhead_data[0], end='\r')
    print(warhead_data[1], end='\r')
    print(warhead_data[2], end='\r')
    print(warhead_data[3], end='\r')
    print(warhead_data[4], end='\r')
    print(warhead_data[5], end='\r')

    print("room_data")
    print(room_data[0], end='\r')
    print(room_data[1], end='\r')
    print(room_data[2], end='\r')
    print(room_data[3], end='\r')
    print(room_data[4], end='\r')
    print(room_data[5], end='\r')

    # try to write to robot file
    try:
        with open(robot_file_path, 'w') as file:
            file.writelines(robot_data)
    except PermissionError:
        return
    
    # try to write to warhead file
    try:
        with open(warhead_file_path, 'w') as file:
            file.writelines(warhead_data)
    except PermissionError:
        return
    
    # try to write to room file
    try:
        with open(room_file_path, 'w') as file:
            file.writelines(room_data)
    except PermissionError:
        return

    # return frame with the axes of all found markers
    return frame

if __name__ == '__main__':

    ap = argparse.ArgumentParser()
    ap.add_argument("-k", "--K_Matrix", required=False, help="Path to calibration matrix (numpy file)", default="C:\\Users\\PrincetonVR\\Desktop\\robotics-nuclear-disarmament-code\\ArUCo-Markers-Pose-Estimation-Generation-Python\\calibration_matrix.npy")
    ap.add_argument("-d", "--D_Coeff", required=False, help="Path to distortion coefficients (numpy file)", default="C:\\Users\\PrincetonVR\\Desktop\\robotics-nuclear-disarmament-code\\ArUCo-Markers-Pose-Estimation-Generation-Python\\distortion_coefficients.npy")
    ap.add_argument("-t", "--type", type=str, help="Type of ArUCo tag to detect", default="DICT_4X4_100")
    args = vars(ap.parse_args())

    if ARUCO_DICT.get(args["type"], None) is None:
        print(f"ArUCo tag type '{args['type']}' is not supported")
        sys.exit(0)

    aruco_dict_type = ARUCO_DICT[args["type"]]
    calibration_matrix_path = args["K_Matrix"]
    distortion_coefficients_path = args["D_Coeff"]
    
    k = np.load(calibration_matrix_path)
    d = np.load(distortion_coefficients_path)

    video = cv2.VideoCapture(0)
    time.sleep(2.0)
    
    i = 0
    while i >= 0:
        ret, frame = video.read()

        if not ret:
            break
        
        output = pose_estimation(frame, aruco_dict_type, k, d)

        try:
            cv2.imshow('Estimated Pose', output)
        except Exception:
            continue

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print("q")
            break
        i += 1

    video.release()
    cv2.destroyAllWindows()