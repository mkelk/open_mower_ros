#!/usr/bin/env python3

# ##################################################################
# Low-level control of Cherokey bot using rosserial
# ##################################################################

import math

import rospy
from std_msgs.msg import String
from std_msgs.msg import UInt16
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from cherokey_msgs.msg import Ticks

class Mowgli_encoder():
    def __init__(self):
        self._latest_raw_reading = 0
        self._currentValue = 1000
    def update(self, raw_reading, speed):
        if raw_reading != 0: 
            # movement, record directed change
            # 0 means no movement OR a reset - in both cases, do not change value
            dir = 1 if speed > 0 else -1
            self._currentValue += dir * (raw_reading - self._latest_raw_reading)
        self._latest_raw_reading = raw_reading
    def get_ticks(self):
        return self._currentValue
        
left_encoder = Mowgli_encoder()
right_encoder = Mowgli_encoder()

left_speed = 0
right_speed = 0


def _rover_cmd_vel_callback(msg):
    rospy.logdebug(f"/rover/cmd_vel received: {msg} - sending to mowgli")
    pub_mowgli_cmd_vel.publish(msg) 

def _mowgli_left_speed_callback(msg):
    global left_speed
    left_speed = msg.data

def _mowgli_right_speed_callback(msg):
    global right_speed
    right_speed = msg.data

def publish_ticks():
    global left_encoder
    global right_encoder
    ticks = Ticks(left_encoder.get_ticks(), right_encoder.get_ticks())
    pub_ticks.publish(ticks)

def _mowgli_left_encoder_callback(msg):
    rospy.logdebug(f"/mowgli/left_encoder_val received: {msg}")
    global left_speed
    global left_encoder
    raw_encoder_reading = msg.data
    left_encoder.update(raw_encoder_reading, left_speed)
    publish_ticks()

def _mowgli_right_encoder_callback(msg):
    rospy.logdebug(f"/mowgli/right_encoder_val received: {msg}")
    global right_speed
    global right_encoder
    raw_encoder_reading = msg.data
    right_encoder.update(raw_encoder_reading, right_speed)
    publish_ticks()


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

    # listen for /mowgli/left_speed_val from mowgli
    _mowgli_left_speed_subscription = rospy.Subscriber('/mowgli/left_speed_val', Float32, _mowgli_left_speed_callback, queue_size=10)
    # listen for /mowgli/left_speed_val from mowgli
    _mowgli_right_speed_subscription = rospy.Subscriber('/mowgli/right_speed_val', Float32, _mowgli_right_speed_callback, queue_size=10)

    # listen for /cmd_vel from OpenMower
    _cmd_vel_subscription = rospy.Subscriber('/rover/cmd_vel', Twist, _rover_cmd_vel_callback, queue_size=10)


    rospy.spin()

    rospy.loginfo("mowgli is ending")

