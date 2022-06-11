#!/usr/bin/env python3

# ##################################################################
# Low-level control of Cherokey bot using rosserial
# ##################################################################

import math

import rospy
from std_msgs.msg import String
from std_msgs.msg import UInt16
from geometry_msgs.msg import Twist
from cherokey_msgs.msg import Ticks

left_encoder_offset = 0
left_encoder_latest_read = 0
right_encoder_offset = 0
right_encoder_latest_read = 0
ticks = Ticks(0,0)

def _rover_cmd_vel_callback(msg):
    rospy.logdebug(f"/rover/cmd_vel received: {msg} - sending to mowgli")
    pub_mowgli_cmd_vel.publish(msg) 

def _mowgli_left_encoder_callback(msg):
    rospy.logdebug(f"/mowgli/left_encoder_val received: {msg}")
    global left_encoder_offset
    global left_encoder_latest_read
    global ticks
    current = msg.data
    if current == 0 and not left_encoder_latest_read == 0:
        # fresh revert to zero, remember new offset
        left_encoder_offset = left_encoder_offset + left_encoder_latest_read
        left_encoder_latest_read = 0
    if current !=0:
        left_encoder_latest_read = current
    ticks.ticksLeft = left_encoder_offset + current
    pub_ticks.publish(ticks)

def _mowgli_right_encoder_callback(msg):
    rospy.logdebug(f"/mowgli/right_encoder_val received: {msg}")
    global right_encoder_offset
    global right_encoder_latest_read
    global ticks
    current = msg.data
    if current == 0 and not right_encoder_latest_read == 0:
        # fresh revert to zero, remember new offset
        right_encoder_offset = right_encoder_offset + right_encoder_latest_read
        right_encoder_latest_read = 0
    if current !=0:
        right_encoder_latest_read = current
    ticks.ticksRight = right_encoder_offset + current
    pub_ticks.publish(ticks)


if __name__ == '__main__':
    rospy.loginfo("mowgli is starting")

    global pub_mowgli_cmd_vel
    global pub_ticks

    # start node
    node = rospy.init_node("mowgli")
    
    # create and start publishers
    # publisher for sending data over rosserial to Mowgli
    pub_mowgli_cmd_vel = rospy.Publisher("/mowgli/cmd_vel", Twist, queue_size=10)
    # start publisher for sending Ticks on to OpenMower framework
    pub_ticks = rospy.Publisher("/rover/ticks", Ticks, queue_size=10)

    # create and start subscribers
    # listen for /cmd_vel from OpenMower
    _cmd_vel_subscription = rospy.Subscriber('/rover/cmd_vel', Twist, _rover_cmd_vel_callback, queue_size=10)
    # listen for /mowgli/left_encoder_val from mowgli
    _mowgli_left_encoder_subscription = rospy.Subscriber('/mowgli/left_encoder_val', UInt16, _mowgli_left_encoder_callback, queue_size=10)
    # listen for /mowgli/right_encoder_val from mowgli
    _mowgli_right_encoder_subscription = rospy.Subscriber('/mowgli/right_encoder_val', UInt16, _mowgli_right_encoder_callback, queue_size=10)

    # start subscriber to listen for ticks from Mowgli
    #sub_ticks = rospy.Subscriber('/rover_ll/ticks', Ticks, ticks_callback, queue_size=10)




    # # do some basic tests
    # cherokey.speed = 0.0 # in meters/sec
    # cherokey._set_motor_speeds()
    # rospy.sleep(2.0)

    # cherokey.speed = 0.0 # in meters/sec
    # cherokey._set_motor_speeds()
    # rospy.sleep(10.0)

    # rospy.loginfo("normal setting")
    # cherokey.speed = 0.25
    # cherokey.spin = 0.0
    # cherokey._set_motor_speeds()
    # rospy.sleep(4.0)

    # cherokey.speed = 0.0 # in meters/sec
    # cherokey._set_motor_speeds()
    # rospy.sleep(1.0)


    # rospy.loginfo("problem setting")
    # cherokey.speed = -0.5 
    # cherokey.spin = -0.16666666666666666 
    # cherokey._set_motor_speeds()
    # rospy.sleep(5)

    # rospy.loginfo("stop")
    # cherokey.speed = 0 # in meters/sec
    # cherokey.spin = 0.0
    # cherokey._set_motor_speeds()
    # rospy.sleep(1.0)


    rospy.spin()

    rospy.loginfo("mowgli is ending")

