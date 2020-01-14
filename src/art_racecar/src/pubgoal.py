#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped as PS
import time
import os
import json

rospy.init_node('goal_publisher')

data = {}

goal = PS()
seq = 0
print os.getcwd()
def pub_gloal():
	if not os.path.exists('goal.json'):
		print 'No Goal Got yet!'
		listener()
		return
	pub = rospy.Publisher('~/move_base_simple/goal', PS, queue_size=1)
	with open('goal.json','r') as a:
		data = json.loads(a.read())
		t = time.time()
		goal.header.stamp.secs = int(t)
		goal.header.stamp.nsecs = int((t - int(t))*10**9)
		goal.header.frame_id = 'map'
		goal.pose.position.x = data['x']
		goal.pose.position.y = data['y']

		goal.pose.orientation.z = data['z']
		goal.pose.orientation.w = data['w']	
	for i in range(2):	
		pub.publish(goal)
		print goal
		time.sleep(1)
	rospy.loginfo('Done')

def	callback(p):
	rospy.loginfo(p)
	with open('goal.json','w') as a:
		data['x'] = p.pose.position.x
		data['y'] = p.pose.position.y
		data['z'] = p.pose.orientation.z
		data['w'] = p.pose.orientation.w
		a.write(json.dumps(data))
	rospy.loginfo('Goal received and saved')
	exit()

def	listener():
	rospy.Subscriber('/move_base_simple/goal',PS,callback)
	rospy.spin()

if __name__=="__main__":
	pub_gloal()

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