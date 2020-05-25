from __future__ import print_function, division
from geometry_msgs.msg import Twist, Vector3
import math
import time
import sys

rotationComplete = False
translationComplete = False

import math 

max_angular = math.pi/8

def go_home(timer, found, point, sameSideCoef, center):

    global rotationComplete
    global translationComplete

    vel_rot = Twist(Vector3(0,0,0), Vector3(0,0,max_angular))
    zero = Twist(Vector3(0,0,0), Vector3(0,0,0))

    sleep_rot = abs(math.pi/max_angular)

    if rotationComplete == False:
        vel = vel_rot
        sleeptime = sleep_rot
        rotationComplete = True
        goHome = True
    
    elif rotationComplete == True and translationComplete == False:
        if not found:

            translationComplete = False
            goHome = True
            sleeptime = 0.1
            

            if sameSideCoef == "Different":
        
                if (point[0] > center[0]):
                    vel = Twist(Vector3(0.15,0,0), Vector3(0,0,-0.1))      

                if (point[0] < center[0]):
                    vel = Twist(Vector3(0.15,0,0), Vector3(0,0,0.1))

                if (abs(point[0] - center[0]) < 10):
                    vel = Twist(Vector3(0.2,0,0), Vector3(0,0,0))

            
            if sameSideCoef == "Negative":

                vel = Twist(Vector3(0.15,0,0), Vector3(0,0,0.15))

            if sameSideCoef == "Positive":

                vel = Twist(Vector3(0.15,0,0), Vector3(0,0,-0.15))
        else:
            translationComplete = True
            goHome = True
            vel = zero
            sleeptime = 0.1

    else:
        vel = zero
        sleeptime = 0.1
        goHome = False
    
    return vel, goHome, rotationComplete, sleeptime
     