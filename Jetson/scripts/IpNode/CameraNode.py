#! /usr/bin/python

import rospy
#import constants
from Jetson.msg import blkData
from Jetson.msg import toCam
from Jetson.msg import bot
from std_msgs.msg import String
import time
import run

pub = rospy.Publisher("blockColors", blkData, queue_size=10)

def callback(data):
	global pos1
	global pos2
	global pos3
	global mode
	pos1=data.pos1
	pos2=data.pos2
	pos3=data.pos3
	#rospy.loginfo("I heard x %s",data.x)
	#rospy.loginfo("I heard x %s",data.y)
	#rospy.loginfo("I heard x %s",data.z)
	print(data)
def callbackD(data):
	print("Request Received to the Camera Node for Location:",data)
	#Process the Data depending upon the Location
	x,y,z=run.give_one_letter()
	msg = blkData()
	msg.mode = 1
	msg.pos1 = x
	msg.pos2 = y
	msg.pos3 = z
	pub.publish(msg)



def listener():
	rospy.init_node("CamNode",anonymous=True)
	rospy.Subscriber("camTopic",toCam,callbackD)
	rospy.Subscriber("blockColors",blkData,callback)

	while not rospy.is_shutdown():
		rospy.spin()

if __name__ == "__main__":
	listener()          #Run the Listener Node
