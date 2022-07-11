#!/usr/bin/env python3

from time import sleep
import rospy
import pypot.robot
from pr2_controllers_msgs import JointTrajectoryAction
import json
from pypot.robot.config import ergo_robot_config
from trajectory_msgs.msg import JointTrajectoryPoint


with open('../pypot_ressources/braccio.json','w') as f:
    json.dump(ergo_robot_config, f, indent=2)

braccio = pypot.robot.from_json("../pypot_ressources/braccio.json")

pub = rospy.Publisher('robot_current_pos',JointTrajectoryAction, queue_size=5)

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

rospy.init_node('Robot_COntroller', anonymous = True)
rospy.Subscriber('robot_target_pos',JointTrajectoryAction,execTrajectory)

rospy.spin()
