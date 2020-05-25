#! /usr/bin/env python
# -*- coding:utf-8 -*-

from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped

def Go_to_creeper (media, mode, centro, dist):

	vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
	print(dist)

	if len(media) != 0 and len(centro) != 0:

		if mode != "Aproach started" and mode != "In front of object":
			if (media[0] > centro[0]):
				vel = Twist(Vector3(0.1,0,0), Vector3(0,0,-0.1))
				mode = "Searching"

			if (media[0] < centro[0]):
				vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0.1))
				mode = "Searching"

			if (abs(media[0] - centro[0]) < 10):
				vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0))
				mode = "Tracking"

		if dist > 1.5 and mode == "Aproach started":
			mode = "Searching"

		if dist < 1.5 and (mode == "Tracking" or mode == "Aproach started"):
			vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0))
			mode = "Aproach started"

		if dist < 0.35 and (mode == "In front of object" or mode == "Aproach started"):
			vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
			mode = "In front of object"
	
	return vel, mode
