#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan as LS
import time
import os
import json
import matplotlib.pyplot as plt
from math import *1
import cv2
rospy.init_node('scan_analysis')
cap = cv2.VideoCapture(0)

if not cap.isOpened():
	print('closed')
	exit()
cap.set(3, 2560);
cap.set(4, 720);
vw = cv2.VideoWriter('/home/sz/ooo.avi', cv2.VideoWriter_fourcc('X','V','I','D'),10,(1280,720))
def	callback(p):
	_,img = cap.read()
	img = img[0:720,0:1280]
	imin = 0
	ranges = list(p.ranges)
	cnt = 40
	for i in range(180):
		i = 179 - i
		if i <= 89:
			j = i+270
		else:
			j = i-90
		if ranges[j] == "inf":
			ranges[j] = 10
		ranges[i] = ranges[j]
		#print "i",i,"x:",ranges[i]*cos(i/180.0*3.1415926),"y:",ranges[i]*sin(i/180.0*3.1415926)
	for i in range(1,179):
		if(ranges[i-1] - ranges[i]>0.5 and ranges[i]<3.5):
			imin = 1
			st = i
		if imin == 1:
			if(ranges[i+1] - ranges[i]>0.5 and ranges[i]<3.5):
				imin = 0
				if(i == st):
					return
				if(i - st > 0.15*360/(7*3.1415926)+2):
					continue
				img = cv2.putText(img,'st:'+str(st)+' end:'+str(i)+' len:'+str(ranges[i]),(50, cnt),cv2.FONT_HERSHEY_SIMPLEX,5,(255, 255, 255),12)
				cnt += 5
				#print "start: "+str(st)+"end: "+str(i)+" len: "+str(ranges[i])
				#print "x:",ranges[i]*cos(i/180.0*3.1415926),"y:",ranges[i]*sin(i/180.0*3.1415926)
	# plt.figure()
	# plt.plot(ranges[0:180])
	# plt.show()
	# print 'l:',l
	# print 'r:',r
	cv2.imshow('sh',img)
	vw.write(img)
	print('wri')

if __name__=="__main__":
	rospy.Subscriber('/scan',LS,callback)
	rospy.spin()
	print 'end'
	cap.release()
	vw.release()
"""
header: 
  seq: 2
  stamp: 
    secs: 195
    nsecs: 105000000
  frame_id: "map"
pose: 
  position: 
    x: -5.29012584686
    y: -5.16840314865
    z: 0.0
  orientation: 
    x: 0.0
    y: 0.0
    z: 0.88022465636
    w: 0.474557219244
"""