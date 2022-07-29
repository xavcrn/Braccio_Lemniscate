#!/usr/bin/env python3

import os,sys,time
import rospy
from braccio.msg import egg_angles
from std_msgs.msg import String

def resize(x, ymin,ymax):
	a = ( ymax - ymin ) / 360
	b = ymin + a * 180
	return int(a*x + b)

"""
def egg_activation(egg_order):
    global egg_enable
    egg_enable = egg_order.data
"""
pid = os.fork()

if pid == 0 :
	os.system("sudo /home/poppy/catkin_ws/src/braccio/receiver/receive")

else :
	egg_enable = True
	pub = rospy.Publisher('egg_angles', egg_angles, queue_size = 10)
	#rospy.Subscriber("egg_activation", Bool, egg_activation)
	rospy.init_node("egg_receiver")
	time.sleep(0.5)
	rospy.loginfo("receiver : Opening pipe for reading")
	pipe = os.open("/home/poppy/catkin_ws/src/braccio/receiver/angles.pipe", os.O_RDONLY)
	rospy.loginfo("receiver : Pipe opened for reading")

	angles = egg_angles()

	lastTime = rospy.get_rostime().nsecs

	angles.m0 = 0
	rate = rospy.Rate(20)

 	
	angles.m1 = 0
	angles.m2 = 0
	angles.m3 = 0
	angles.m4 = 0
	angles.m5 = 0

	sX = [0,0,0]
	sY = [0,0,0]
	nX = [0,0,0]
	nY = [0,0,0]

	while not rospy.is_shutdown():
		while egg_enable:
			# we can't only read at 20 Hz because we only can read eggs one by one
			"""
			ID = int(os.read(pipe,2)[0:1])
			X  = int(os.read(pipe,4)[0:3])
			Y  = int(os.read(pipe,4)[0:3])
			"""
			ID = int.from_bytes(os.read(pipe,1),"little",signed=True)
			X  = int.from_bytes(os.read(pipe,2),"little",signed=True)
			Y  = int.from_bytes(os.read(pipe,2),"little",signed=True)

			rospy.loginfo("receiver : ID={}, X={}, Y={}".format(ID,X,Y))
			if ID == 1:
				sX[0] += X
				sY[0] += Y
				nX[0] += 1
				nY[0] += 1
			elif ID == 2:
				sX[1] += X
				sY[1] += Y			
				nX[1] += 1
				nY[1] += 1
			elif ID == 3:
				sX[2] += X
				sY[2] += Y
				nX[2] += 1
				nY[2] += 1

			currentTime = rospy.get_rostime().nsecs
			if currentTime < lastTime :
				lastTime = currentTime

			if currentTime - lastTime > 50000000 : #20Hz
				# send mean values of motor angles during the last 0.05s
				if nX[0] != 0:				
					angles.m0 = int(resize(sX[0]/nX[0], -180, 180))
				if nY[0] != 0:
					angles.m1 = int(resize(sY[0]/nY[0], -130, 130))
				if nX[1] != 0:
					angles.m2 = int(resize(sX[1]/nX[1], -120, 120))
				if nY[1] != 0:
					angles.m3 = int(resize(sY[1]/nY[1], -100, 100))
				if nX[2] != 0:
					angles.m4 = int(resize(sX[2]/nX[2], -180, 180))
				if nY[2] != 0:
					angles.m5 = int(resize(sY[2]/nY[2],    0,  90))
				
				pub.publish(angles)
				sX = [0,0,0]
				sY = [0,0,0]
				nX = [0,0,0]
				nY = [0,0,0]
				lastTime += 50000000

				#rospy.loginfo("receiver : Angles sent : %4d %4d %4d %4d %4d %4d",angles.m0, angles.m1, angles.m2, angles.m3, angles.m4, angles.m5 )
		rate.sleep()
	rospy.loginfo("receiver : Shutting down")
