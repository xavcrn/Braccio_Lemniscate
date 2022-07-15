#!/usr/bin/env python3
import rospy
from pypot.robot import from_json
from time import sleep

from braccio.msg import egg_angles
from braccio.srv import creation, creationResponse
from std_msgs.msg import Bool,String
from pypot.primitive.move import MoveRecorder, Move, MovePlayer

## Define all global variables
MOVE_PATH = "/home/poppy/catkin_ws/src/braccio/moves/"
# Define sleeping position
sleeping_position = [0,4,-109,-72,0,0]
# Define speeds
egg_motor_speed = [10,10,10,10,10,30]
max_motor_speed = [90,100,110,120,130,90]
safety_motor_speed = [35,35,35,35,35,35]
reach_initial_speed = [60,60,60,60,60,60]
# Disable egg_control by default
egg_enable = False
egg_position = [0,0,0,0,0,0,0]

# Load braccio.json configuration
braccio = from_json("/home/poppy/catkin_ws/src/braccio/pypot_ressources/braccio.json")
# Initialise move recorder
move_recorder = MoveRecorder(braccio, 50, braccio.motors)

rospy.init_node("pypot_controler")
pub = rospy.Publisher("movement_playing", Bool,queue_size=5)

def set_speeds(speeds):
    for k in range(6):
        braccio.motors[k].moving_speed = speeds[k]

def wait_for_position():
    for m in braccio.arm:
        while abs(m.goal_position - m.present_position) > 1:
            sleep(0.2)

def initial():
    braccio.compliant = False
    set_speeds(safety_motor_speed)
    for m in braccio.arm:
        m.goal_position = 0
    wait_for_position()

def go_sleep():
    rospy.loginfo("Go to initial position then to sleeping position")
    initial()
    set_speeds(safety_motor_speed)
    for k in range(6):
        braccio.arm[k].goal_position = sleeping_position[k]
    wait_for_position()
    braccio.compliant = True

def record(request):
    braccio.compliant = True
    move_recorder.start()
    sleep(request.duration)
    move_recorder.stop()
    move_name = MOVE_PATH + request.move_name 
    with open(move_name, 'w') as f:
        move_recorder.move.save(f)
    return creationResponse(True)

def play(move):
    # If want to go to initial position
    if move.data == "initial":
        initial()
        pub.publish(False)
        return
    # If want to go to sleep position
    if move.data == "sleep":
        go_sleep()
        pub.publish(False)
        return
    # If want to play recorded moves
    # Open move file
    move_name = MOVE_PATH + move.data
    with open(move_name) as f:
        m = Move.load(f)
    # Go to first position before begining movement
    set_speeds(reach_initial_speed)
    braccio.compliant = False
    braccio.arm[0].goal_position = m.positions()[0]['m0'][0]
    braccio.arm[1].goal_position = m.positions()[0]['m1'][0]
    braccio.arm[2].goal_position = m.positions()[0]['m2'][0]
    braccio.arm[3].goal_position = m.positions()[0]['m3'][0]
    braccio.arm[4].goal_position = m.positions()[0]['m4'][0]
    braccio.arm[5].goal_position = m.positions()[0]['m5'][0]
    wait_for_position()
    # Play movement
    set_speeds(max_motor_speed)
    move_player = MovePlayer(braccio, m)
    pub.publish(True)
    move_player.start()
    # Wait for movement to end
    sleep(move_player.duration() + 0.1)
    pub.publish(False)

def egg_activation(egg_order):
    egg_enable = egg_order.data
    
def egg_set_position(egg_target):
    for k in range(6):
        egg_position[k] = egg_target[k]

# Initialize all subscribers
rospy.Subscriber("egg_activation", Bool, egg_activation)
rospy.Subscriber("play_move", String, play)
rospy.Subscriber("eggs_angles", egg_angles, egg_set_position)

# Start creation service
s = rospy.Service("create_move", creation, record)

rospy.loginfo("Braccio arm Launched")

initial()

rospy.loginfo("Initial position reached")

rate = rospy.rate(20)

while not rospy.is_shutdown():
    if egg_enable:
        set_speeds(egg_motor_speed)
        for k in range(6):
            braccio.motors[k].compliant = False
            braccio.motors[k].goal_position = egg_position[k]
    rate.sleep()

# Test if the robot is in sleeping position before leaving
sleeping = True
for k in range(6):
    if braccio.motors[k].goal_position != sleeping_position[k]:
        sleeping = False
        break
if not sleeping:
    go_sleep()

rospy.loginfo("Braccio is now sleeping")