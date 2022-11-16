'''
Sample Usage:-
python pose_estimation.py --K_Matrix calibration_matrix.npy --D_Coeff distortion_coefficients.npy --type DICT_5X5_100
'''


import numpy as np
import cv2
from datetime import datetime
import sys
from utils import ARUCO_DICT
import argparse
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

    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=parameters)
    
    robot_file_path = "C:\\Users\\PrincetonVR\\Documents\\Unreal Projects\\MyProject_VRTemplate_Robots\\Content\\Files\\robot_pose.txt"
    try:
        with open(robot_file_path, 'r') as file:
            robot_data = file.readlines()
    except PermissionError:
        return

    warhead_file_path = "C:\\Users\\PrincetonVR\\Documents\\Unreal Projects\\MyProject_VRTemplate_Robots\\Content\\Files\\warhead_pose.txt"
    try:
        with open(warhead_file_path, 'r') as file:
            warhead_data = file.readlines()
    except PermissionError:
        return

    # If markers are detected
    if len(corners) > 0:
        for i in range(0, len(ids)):
            # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients,
                                                                       distortion_coefficients)
            
            curr_id = ids[i][0]

            str_to_write = str(curr_id)
            for element in tvec[0][0]:
                str_to_write = str_to_write + "," + str(element)
            for element in rvec[0][0]:
                str_to_write = str_to_write + "," + str(element)
            current_date = datetime.now()
            str_to_write += "," + current_date.isoformat() + ";\n"
            
            if curr_id >= 0 and curr_id < 6:
                robot_data[curr_id] = str_to_write
            if curr_id >= 6 and curr_id < 12:
                warhead_data[curr_id - 6] = str_to_write

            # Draw a square around the markers
            cv2.aruco.drawDetectedMarkers(frame, corners) 

            # Draw Axis
            cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)  

    try:
        with open(robot_file_path, 'w') as file:
            file.writelines(robot_data)
    except PermissionError:
        return
    
    try:
        with open(warhead_file_path, 'w') as file:
            file.writelines(warhead_data)
    except PermissionError:
        return

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
        # time.sleep(0.1)

    video.release()
    cv2.destroyAllWindows()