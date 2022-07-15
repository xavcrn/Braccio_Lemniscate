#!/usr/bin/env python3

import pypot.dynamixel
from braccio.msg import egg_angles
from braccio.srv import creation
import rospy

rospy.init_node("controler")

#ports = pypot.dynamixel.get_available_ports()
dxl_io = pypot.dynamixel.DxlIO("/dev/ttyACM0")

rospy.loginfo(dxl_io.get_present_voltage((0,1,2,3,4,5)))
dxl_io.get_present_voltage((0,1,2,3,4,5))

speeds = [10,10,10,10,10,10]
speedsmax = [90,100,110,120,130,90]

dxl_io.set_moving_speed({0: speeds[0]})
dxl_io.set_moving_speed({1: speeds[1]})
dxl_io.set_moving_speed({2: speeds[2]})
dxl_io.set_moving_speed({3: speeds[3]})
dxl_io.set_moving_speed({4: speeds[4]})
dxl_io.set_moving_speed({5: speeds[5]})

dxl_io.set_goal_position({0: 0})
dxl_io.set_goal_position({1: -43})
dxl_io.set_goal_position({2: 0})
dxl_io.set_goal_position({3: 0})
dxl_io.set_goal_position({4: 0})
dxl_io.set_goal_position({5: 3})

def eggRemote(angles):
    dxl_io.set_goal_position({0: angles.m0})
    dxl_io.set_goal_position({1: angles.m1 -43})
    dxl_io.set_goal_position({2: angles.m2})
    dxl_io.set_goal_position({3: angles.m3})
    dxl_io.set_goal_position({4: angles.m4})
    dxl_io.set_goal_position({5: angles.m5 +3})

rospy.sleep(2)

rospy.Subscriber("eggs_angles", egg_angles, eggRemote)

rospy.spin()
