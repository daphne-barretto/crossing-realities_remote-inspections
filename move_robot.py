#
# Licensed under 3-Clause BSD license available in the License file. Copyright (c) 2021 iRobot Corporation. All rights reserved.
#

from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

backend = Bluetooth()
robot = Root(backend)

@event(robot.when_play)
async def move_from_text(robot):
    while True:
        print('move_from_text')

        move_robot_path = "/Users/testadmin/Desktop/code/mae418/ArUCo-Markers-Pose-Estimation-Generation-Python/move_robot.txt"
        with open(move_robot_path, 'r') as file:
            move_robot_data = [int(x) for x in file.readlines()[0].split(",")]

        print(move_robot_data)

        forward_backward_change, turn_left_change, turn_right_change = move_robot_data
        if forward_backward_change != 0:
            await robot.move(forward_backward_change)
        if turn_left_change != 0:
            await robot.turn_left(turn_left_change)
        elif turn_right_change != 0:
            await robot.turn_right(turn_right_change)

@event(robot.when_bumped, [])
async def bump(robot):
    while True:
        print('bump')


robot.play()