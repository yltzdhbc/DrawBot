'''
@Author: ylt
@Date: 2019-10-29 18:58:43
@LastEditors: ylt
@LastEditTime: 2019-11-23 16:08:50
@FilePath: \WebServer\ACTION.py
'''
#  ROBOT STEP CONSTANTS
#  STEPSMETER = 11428; STEPS/METERS      default: 11428
#  STEPSTURN = 4720; STEPS/TURN (360 deg) default:4720
import time
import thread
from ROBOT_Class import ROBOT  # Import CLASS to control BROBOT

myRobot = ROBOT()  #init the robot
# myRobot.turn(90)
scale = 20


def N0():
    myRobot.line(2 * scale)
    myRobot.turn(90)
    myRobot.line(-2 * scale)
    myRobot.turn(-90)
    myRobot.line(4 * scale)
    myRobot.turn(90)
    myRobot.line(2 * scale)
    myRobot.turn(90)
    myRobot.line(4 * scale)
    myRobot.turn(90)
    myRobot.line(-2 * scale)
    myRobot.turn(-90)


N0()
