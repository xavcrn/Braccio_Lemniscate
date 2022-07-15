#!/usr/bin/env python3
from time import sleep
import rospy
import pypot.robot as robot
import json
from trajectory_msgs.msg import JointTrajectoryPoint
from braccio.msg import egg_angles


braccio = robot.from_json("/home/poppy/catkin_ws/src/braccio/pypot_ressources/braccio.json")

rospy.loginfo("json config successfully loaded")

pub = rospy.Publisher('robot_current_pos',JointTrajectoryAction, queue_size=5)

egg_control = True

def chgtSource(source):
    egg_control = source

def pubCurrentPos():
    currentPos = JointTrajectoryPoint()
    for k in braccio.motors :
        currentPos.positions.append(k.current_position)
    pub.publish(currentPos)

def execTrajectory(goal):
    currentTime = 0
    for traj in goal.trajectory.points :
        for k in range(0,traj.points.size):
            braccio.motors[k].goal_position = traj.positions[k]
        sleep(traj.time_from_start - currentTime)
        currentTime += traj.time_from_start
    pubCurrentPos()

#angles = egg_angles()

def eggsControl(anglesIN):
    robot.motors[0].goal_position = anglesIN.m0
    robot.motors[1].goal_position = anglesIN.m1
    robot.motors[2].goal_position = anglesIN.m2
    robot.motors[3].goal_position = anglesIN.m3
    robot.motors[4].goal_position = anglesIN.m4
    robot.motors[5].goal_position = anglesIN.m5

rospy.init_node('Robot_Controller', anonymous = True)
rospy.Subscriber('robot_target_pos',JointTrajectoryAction,execTrajectory)
rospy.Subscriber('command_source',bool,chgtSource)
rospy.Subscriber('eggs_angles',egg_angles,eggsControl)

rospy.spin()
