#!/usr/bin/env python3
import os,sys,time
import rospy

pid = os.fork()

if pid > 0 :
	time.sleep(0.5)
	print("Child PID : ",pid)
	print("Opening pipe for reading")
	pipe = os.open("/home/poppy/catkin_ws/src/braccio/receiver/temp/transferToPython", os.O_RDONLY)
	print("Pipe opened for reading")

	ID   = int(os.read(pipe,2)[0:1])
	minX = int(os.read(pipe,4)[0:3])
	minY = int(os.read(pipe,4)[0:3])

	maxX = minX
	maxY = minY

	n0 = 0
	n1 = 0
	n2 = 0

	print(ID)

	for k in range(0,40):
		for k in range(0,40):
			ID = int(os.read(pipe,2)[0:1])
			X  = int(os.read(pipe,4)[0:3])
			Y  = int(os.read(pipe,4)[0:3])
			if Y > maxY :
				maxY = Y
			else :
				if Y < minY :
					minY = Y
			if X > maxX :
				maxX = X
			else :
				if X < minX :
					minX = X
		print("X : [", minX, ";", maxX, "]")
		print("Y : [", minY, ";", maxY, "]")
	
#	while True :
#        
#		eggId = os.read(pipe, 2)
#		receptionId = int(eggId[0:1])
#		#("ID =", receptionId)
		
#		cooX = os.read(pipe, 4)
#		receptionX = int(cooX[0:3])
#		print("X =", receptionX)
		
#		cooY = os.read(pipe, 4)
#		receptionY = int(cooY[0:3])
#		#print("Y =", receptionY)
	os.close(pipe)
		
else :
	os.system("sudo /home/poppy/catkin_ws/src/braccio/receiver/receive")

