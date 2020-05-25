from __future__ import print_function, division
import rospy
import numpy as np
import cv2
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
import math
import time
from tf import transformations
import sys

rotationComplete = False
translationComplete = False

### Funcoes da solucao
import math 

max_angular = math.pi/8

def go_home(timer, found):

    global rotationComplete
    global translationComplete

    vel_rot = Twist(Vector3(0,0,0), Vector3(0,0,max_angular))
    zero = Twist(Vector3(0,0,0), Vector3(0,0,0))

    sleep_rot = abs(math.pi/max_angular)

    if rotationComplete == False:
        print(vel_rot, "\n",  sleep_rot)
        vel = vel_rot
        sleeptime = sleep_rot
        rotationComplete = True
        goHome = True
    
    elif rotationComplete == True and translationComplete == False:
        if not found:
            vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
            sleeptime = 0.1
            goHome = True
            translationComplete = False
        else:
            translationComplete = True
            goHome = True
            vel = zero
            sleeptime = 0.1

    else:
        print("Terminou um ciclo")
        vel = zero
        sleeptime = 0.1
        goHome = False
    
    print("le gohome:" , goHome)
    return vel, goHome, sleeptime
     