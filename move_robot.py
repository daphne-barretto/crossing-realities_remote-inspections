# move_robot.py, Daphne Barretto
# reads the y and x values from a file (assumably written from a joystick), and
# rotates and translates a robot using those values
# adapted from iRobot 26_sing_and_walk.py at https://python.irobot.com/

from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, Create3
import datetime

# robot tuning parameters
JOYSTICK_Y_THRESHOLD = 0.3
JOYSTICK_Y_TRANSLATE_VALUE = 10
JOYSTICK_X_THRESHOLD = 0.3
JOYSTICK_X_ROTATE_VALUE = 10

# joystick file path parameter
joystick_file_path = "C:\\Users\\PrincetonVR\\Documents\\Unreal Projects\\MyProject_VRTemplate_Robots\\Content\\Files\\joystick_values.txt"

# create robot object
backend = Bluetooth()
robot = Create3(backend)

# infinite loop while robot is playing
@event(robot.when_play)
async def move_from_text(robot):
    while True:

        print('move_from_text')
        
        # read and parse move robot data, if available
        try:
            with open(joystick_file_path, 'r') as file:
                move_robot_data = file.readlines()[0].split(",")
                move_robot_data[0] = float(move_robot_data[0])
                move_robot_data[1] = float(move_robot_data[1])
                move_robot_data[2] = datetime.strptime(move_robot_data[2], "%04d.%02d.%02d-%02d.%02d.%02d")
        except PermissionError:
            print("couldn't open")
            continue

        print(move_robot_data)

        # check if data is from the last 1.5 seconds
        if move_robot_data[2] > datetime.datetime.now() - datetime.timedelta(seconds=1.5):
            # parse move robot data into individual y and x values
            joystick_y_value, joystick_x_value = move_robot_data
            # if above the x threshold tuned, rotate the amount tuned
            if abs(joystick_x_value) > JOYSTICK_X_THRESHOLD: 
                if joystick_x_value > 0:
                    await robot.turn_right(JOYSTICK_X_ROTATE_VALUE)
                else:
                    await robot.turn_left(JOYSTICK_X_ROTATE_VALUE)
            # if above the y threshold tuned, translate the amount tuned
            elif abs(joystick_y_value) > JOYSTICK_Y_THRESHOLD: 
                if joystick_y_value > 0:
                    await robot.move(JOYSTICK_Y_TRANSLATE_VALUE)
                else:
                    await robot.move(-JOYSTICK_Y_TRANSLATE_VALUE)

# play the robot
robot.play()