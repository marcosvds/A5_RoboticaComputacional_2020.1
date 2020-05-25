#! /usr/bin/env python
# -*- coding:utf-8 -*-


import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import smach
import smach_ros

def cross(img_rgb, point, color, width,length):
        cv2.line(img_rgb, (int(point[0] - length/2), point[1]),  (int(point[0] + length/2), point[1]), color ,width, length)
        cv2.line(img_rgb, (point[0], int(point[1] - length/2)), (point[0], int(point[1] + length/2)),color ,width, length) 


def identifica_cor(frame, contadorContorno, target, color):

    centro = (frame.shape[1]//2, frame.shape[0]//2)
    maior_contorno = None
    maior_contorno_area = None

    if color == "Verde":
        cor_menor = np.array([50, 50, 50])
        cor_maior = np.array([60, 255, 255])

    if color == "Azul":
        cor_menor = np.array([88, 50, 50])
        cor_maior = np.array([108, 255, 255])

    if color == "Rosa":  
        cor_menor = np.array([140, 50, 50])
        cor_maior = np.array([150, 255, 255])

    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) 
    img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2GRAY)
    segmentado_cor = cv2.inRange(frame_hsv, cor_menor, cor_maior)
    segmentado_cor = cv2.morphologyEx(segmentado_cor,cv2.MORPH_CLOSE,np.ones((7, 7)))

    not_mask = cv2.bitwise_not(segmentado_cor) 
    res = cv2.bitwise_and(img_gray,img_gray, mask = not_mask) 
    res2 = cv2.cvtColor(res, cv2.COLOR_GRAY2RGB) 
    res3 = cv2.bitwise_and(frame,frame, mask= segmentado_cor) 
    final = cv2.bitwise_or(res3, res2)

    contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 

    for cnt in contornos:
        area = cv2.contourArea(cnt)
        maior_contorno = cnt
        maior_contorno_area = area

    if maior_contorno_area < 1000 and maior_contorno_area > 10 :
        contadorContorno += 1
    
    else:
        contadorContorno = 0

    if contadorContorno > 25:
        target = 1
    
    if target:
        if not maior_contorno is None :
            maior_contorno = np.reshape(maior_contorno, (maior_contorno.shape[0], 2))
            media = maior_contorno.mean(axis=0)
            media = media.astype(np.int32)
            cv2.circle(final, (media[0], media[1]), 3, [0, 255, 0], 5)
            cross(final, centro, [0,255,0], 5, 17)
        
        else:
            media = (0, 0)
    
    else:
        media = (0, 0)

    print("cormodule:", target)
    

    return media, centro, maior_contorno_area, final, contadorContorno, target
