#
# Licensed under 3-Clause BSD license available in the License file. Copyright (c) 2021 iRobot Corporation. All rights reserved.
#

from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

JOYSTICK_Y_TRANSLATE_VALUE = 10
JOYSTICK_X_THRESHOLD = 0.5
JOYSTICK_X_ROTATE_VALUE = 10

backend = Bluetooth()
robot = Create3(backend)

@event(robot.when_play)
async def move_from_text(robot):
    while True:
        print('move_from_text')

        joystick_file_path = "C:\\Users\\PrincetonVR\\Documents\\Unreal Projects\\MyProject_VRTemplate_Robots\\Content\\Files\\joystick_values.txt"
        try:
            with open(joystick_file_path, 'r') as file:
                move_robot_data = [float(x) for x in file.readlines()[0].split(",")]
        except PermissionError:
            print("couldn't open")
            continue

        print(move_robot_data)

        joystick_y_value, joystick_x_value = move_robot_data
        if abs(joystick_x_value) > JOYSTICK_X_THRESHOLD:
            if joystick_x_value > 0:
                await robot.turn_right(JOYSTICK_X_ROTATE_VALUE)
            else:
                await robot.turn_left(JOYSTICK_X_ROTATE_VALUE)
        elif abs(joystick_y_value) > 0:
            if joystick_y_value > 0:
                await robot.move(JOYSTICK_Y_TRANSLATE_VALUE)
            else:
                await robot.move(-JOYSTICK_Y_TRANSLATE_VALUE)

@event(robot.when_bumped, [])
async def bump(robot):
    while True:
        print('bump')


robot.play()