#!/usr/bin/env python3
from pypot.robot import from_json
from time import sleep
import rospy
from braccio.msg import egg_angles
from braccio.msg import start_move
from braccio.srv import creation, creationResponse
from std_msgs.msg import Bool
from pypot.primitive.move import MoveRecorder, Move, MovePlayer
from os import listdir
from os.path import isfile, join


rospy.init_node("pypot_controler")
braccio = from_json("/home/poppy/catkin_ws/src/braccio/pypot_ressources/braccio.json")
move_recorder = MoveRecorder(braccio, 50, braccio.motors)


rospy.loginfo("Braccio Launched")

for m in braccio.arm:
    m.compliant = False
    m.moving_speed = 40

def wait_for_position():
    for m in braccio.arm:
        while abs(m.goal_position - m.present_position) > 1:
            sleep(0.2)

def initial():
    for m in braccio.arm:
        m.moving_speed = 40
        m.goal_position = 0
    wait_for_position()

def go_sleep():
    rospy.loginfo("Go to initial position then to sleeping position")
    initial()
    dest = [0,4,-109,-72,0,0]
    for k in range(6):
        braccio.arm[k].goal_position = dest[k]
        braccio.arm[k].moving_speed = 30
    wait_for_position()
    braccio.compliant = True

def record(request):
    move_recorder.start()
    time.sleep(request.duration)
    move_recorder.stop()
    move_name = "/home/poppy/catkin_ws/src/braccio/moves/" + request.move_name 
    with open(move_name, 'w') as f:
        move_recorder.move.save(f)
    return creationResponse(True)

def play(move):
    # Open move file
    move_name = "/home/poppy/catkin_ws/src/braccio/moves/" + move
    with open(move) as f:
        m = Move.load(f)
    # Go to first position before begining movement
    braccio.compliant = False
    braccio.arm[0].goal_position = m.positions()[0]['m0'][0]
    braccio.arm[1].goal_position = m.positions()[0]['m1'][0]
    braccio.arm[2].goal_position = m.positions()[0]['m2'][0]
    braccio.arm[3].goal_position = m.positions()[0]['m3'][0]
    braccio.arm[4].goal_position = m.positions()[0]['m4'][0]
    braccio.arm[5].goal_position = m.positions()[0]['m5'][0]
    wait_for_position()
    # Play movement
    move_player = MovePlayer(braccio, m)
    move_player.start()
    # Wait for movement to end
    sleep(move_player.duration() + 0.1)

rospy.spin()
