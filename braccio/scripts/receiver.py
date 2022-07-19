#!/usr/bin/env python3

import os,sys,time
import rospy
from braccio.msg import egg_angles

def resize(x,xmin,xmax,ymin,ymax):
	if x < xmin :
		x = xmin
	if x > xmax :
		x = xmax
	a = ( ymax - ymin ) / (xmax - xmin)
	b = ymin - a * xmin

	return int(a*x + b)
	

pid = os.fork()

if pid == 0 :
	os.system("sudo /home/poppy/catkin_ws/src/braccio/receiver/receive")

else :
	pub = rospy.Publisher('egg_angles', egg_angles, queue_size = 10)
	rospy.init_node("egg_receiver")
	time.sleep(0.5)
	rospy.loginfo("Opening pipe for reading")
	pipe = os.open("/home/poppy/catkin_ws/src/braccio/receiver/temp/transferToPython", os.O_RDONLY)
	rospy.loginfo("Pipe opened for reading")

	angles = egg_angles()

	lastTime = rospy.get_rostime().nsecs

	sX = [0,0,0]
	sY = [0,0,0]
	nX = [1,1,1]
	nY = [1,1,1]

	while not rospy.is_shutdown():
		ID = int(os.read(pipe,2)[0:1])
		X  = int(os.read(pipe,4)[0:3])
		Y  = int(os.read(pipe,4)[0:3])

		if ID == 0:
			sX[0] += X
			sY[0] += Y
			nX[0] += 1
			nY[0] += 1
		if ID == 1:
			sX[1] += X
			sY[1] += Y
			nX[1] += 1
			nY[1] += 1
		if ID == 3:
			sX[2] += X
			sY[2] += Y
			nX[2] += 1
			nY[2] += 1

		currentTime = rospy.get_rostime().nsecs
		if currentTime < lastTime :
			lastTime = currentTime

		if currentTime - lastTime > 50000000 : #20Hz
			# send mean values of motor angles during the last 0.05s
			
			lastTime += 50000000

			sX = [sX[0]/nX[0],sX[1]/nX[1],sX[2]/nX[2]]
			sY = [sY[0]/nY[0],sY[1]/nY[1],sY[2]/nY[2]]

			angles.m0 = int(resize(sX[0], 240, 510, -180, 180))
			angles.m1 = int(resize(sY[0], 240, 510, -120, 120))
			angles.m2 = int(resize(sX[1], 240, 510, -120, 120))
			angles.m3 = int(resize(sY[1], 240, 510, -100, 100))
			angles.m4 = int(resize(sX[2], 240, 510, -180, 180))
			angles.m5 = int(resize(sY[2], 240, 510,   -90,  0))
			
			#print(int(sX[0]),int(sY[0]),int(sX[1]),int(sY[1]),int(sX[2]),int(sY[2]))

			pub.publish(angles)
			
			nX = [1,1,1]
			nY = [1,1,1]

			#rospy.loginfo("Angles sent : %4d %4d %4d %4d %4d %4d",angles.m0, angles.m1, angles.m2, angles.m3, angles.m4, angles.m5 )

	rospy.loginfo("Shutting down receiver")
