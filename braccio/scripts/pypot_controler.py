#!/usr/bin/env python3
import rospy
from pypot.robot import from_json
from time import sleep

from braccio.msg import egg_angles
from std_msgs.msg import Bool, String
from pypot.primitive.move import MoveRecorder, Move, MovePlayer

## Define all global variables
MOVE_PATH = "/home/poppy/catkin_ws/src/braccio/moves/"
# Define sleeping position
sleeping_position = [0,4,-109,-72,0,0]
# Define speeds
egg_motor_speed = [10,10,10,10,30,30]
max_motor_speed = [90,100,110,120,130,90]
safety_motor_speed = [35,35,35,35,35,35]
reach_initial_speed = [50,50,50,50,50,50]
# Disable egg_control by default
egg_enable = False
egg_position = [0,0,0,0,0,0,0]
# Disable recording
recording = False
move_to_record = ""

# Load braccio.json configuration
braccio = from_json("/home/poppy/catkin_ws/src/braccio/pypot_ressources/braccio.json")

rospy.init_node("pypot_controler")
pub = rospy.Publisher("movement_playing", Bool, queue_size=5)

def set_speeds(speeds):
    for k in range(6):
        braccio.motors[k].moving_speed = speeds[k]

def wait_for_position():
    for m in braccio.arm:
        while abs(m.goal_position - m.present_position) > 2:
            sleep(0.1)

def initial():
    braccio.compliant = False
    set_speeds(safety_motor_speed)
    for m in braccio.arm:
        m.goal_position = 0
    wait_for_position()

def go_sleep():
    initial()
    set_speeds(safety_motor_speed)
    for k in range(6):
        braccio.arm[k].goal_position = sleeping_position[k]
    wait_for_position()
    sleep(0.1)
    braccio.compliant = True

def record(move):
    rospy.loginfo("pypot_controler : Recording movement \"{}\"".format(move))
    move_recorder = MoveRecorder(braccio, 50, braccio.motors)
    braccio.compliant = True    
    move_recorder.start()
    global recording
    while recording:
        sleep(0.05)
    move_recorder.stop()
    braccio.compliant = False
    move_name = MOVE_PATH + move
    with open(move_name, 'w') as f:
        move_recorder.move.save(f)

def play(move):
    # If want to go to initial position
    if move.data == "initial":
        initial()
        rospy.loginfo("pypot_controler : Initial position reached")
        pub.publish(False)
        return
    # If want to go to sleep position
    if move.data == "sleep":
        go_sleep()
        pub.publish(False)
        rospy.loginfo("pypot_controler : Sleeping position reached")
        return
    # If want to play recorded moves
    # Open move file
    move_name = MOVE_PATH + move.data
    with open(move_name) as f:
        m = Move.load(f)
    move_player = MovePlayer(braccio, m)
    # Go to first position before begining movement
    set_speeds(reach_initial_speed)
    braccio.compliant = False
    braccio.arm[0].goal_position = m.positions()[0]['m0'][0]
    braccio.arm[1].goal_position = m.positions()[0]['m1'][0]
    braccio.arm[2].goal_position = m.positions()[0]['m2'][0]
    braccio.arm[3].goal_position = m.positions()[0]['m3'][0]
    braccio.arm[4].goal_position = m.positions()[0]['m4'][0]
    braccio.arm[5].goal_position = m.positions()[0]['m5'][0]
    for m in braccio.arm:
        if m.goal_position < min(m.angle_limit):
            m.goal_position = m.angle_limit[0]
        elif m.goal_position > max(m.angle_limit):
            m.goal_position = m.angle_limit[1]
    wait_for_position()
    # Play movement
    rospy.loginfo("pypot_controler : now playing : \"{}\"".format(move.data))
    set_speeds(max_motor_speed)
    
    pub.publish(True)
    move_player.start()
    # Wait for movement to end
    sleep(move_player.duration() + 0.1)
    pub.publish(False)

def egg_activation(egg_order):
    global egg_enable
    egg_enable = egg_order.data
    
def egg_set_position(egg_target):
    egg_position[0] = egg_target.m0
    egg_position[1] = egg_target.m1
    egg_position[2] = egg_target.m2
    egg_position[3] = egg_target.m3
    egg_position[4] = egg_target.m4
    egg_position[5] = egg_target.m5

def record_ctrl(msg):
    global recording
    global move_to_record
    print(msg)
    if msg.data == "STOP":
        print("msg = STOP")
        recording = False
    elif recording == False:
        print("msg != STOP")
        move_to_record = msg.data
        recording = True

# Initialize all subscribers
rospy.Subscriber("egg_activation", Bool, egg_activation)
rospy.Subscriber("play_move", String, play)
rospy.Subscriber("egg_angles", egg_angles, egg_set_position)
rospy.Subscriber("recording", String, record_ctrl)

# Start creation service
#s = rospy.Service("create_move", creation, record)

rospy.loginfo("pypot_controler : Braccio arm Launched")

rate = rospy.Rate(20)

rospy.loginfo("pypot_controler : Enterring loop")
while not rospy.is_shutdown():
    if recording:
        record(move_to_record)
        print("move recorded")
    elif egg_enable:
        set_speeds(egg_motor_speed)
        for k in range(6):
            braccio.motors[k].goal_position = egg_position[k]
    rate.sleep()

# Test if the robot is in sleeping position before leaving
if not braccio.compliant:
    go_sleep()
    braccio.compliant = True

rospy.loginfo("pypot_controler : Braccio is now sleeping")
