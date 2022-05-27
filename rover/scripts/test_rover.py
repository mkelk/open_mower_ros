#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from cherokey_msgs.msg import Ticks
from ublox_msgs.msg import NavRELPOSNED9

start_time = None
last_ticks_time = None
last_navrelposned_time = None
last_navrelposned = None



def sayProblem(msg):
    rospy.logwarn(f"**** problem ***** {msg}")

def sayOk(msg):
    rospy.loginfo(f"--- ok --- {msg}")

def waitUntilSomeSecondsIn(secs):
    while (rospy.get_rostime() - start_time).to_sec() < secs:
        rospy.loginfo("Sleeping...")
        rospy.sleep(1.0)        

def stdSay(mustbetrue, okmsg, problemmsg):
    if mustbetrue:
        sayOk(okmsg)
    else:
        sayProblem(problemmsg)


def ticks_callback(ticks):
    global last_ticks_time
    last_ticks_time = rospy.get_rostime() 

def testTicks():
    stdSay( (last_ticks_time == None or rospy.get_rostime() - last_ticks_time).to_sec() < 1.0, "Ticks coming in fine from /rover_ll/ticks", "Too long wait for ticks from /rover_ll/ticks")

def navrelposned_callback(navrelposned):
    global last_navrelposned_time
    global last_navrelposned
    last_navrelposned_time = rospy.get_rostime()
    last_navrelposned = navrelposned

def testNavrelposned():
    stdSay( (rospy.get_rostime() - last_navrelposned_time).to_sec() < 2.0, 
        "Navrelposned coming in fine from /ublox/navrelposned", "Too long wait for navrelposned from /ublox/navrelposned")
    acc = last_navrelposned.accLength / 10000
    if (acc > 0.05):
        sayProblem("Last navrelposned accLength > 0.05m")
    flags = last_navrelposned.flags
    gnssFixOK = (flags & 0b0000001)
    diffSoln = (flags & 0b0000010) >> 1
    relPosValid = (flags & 0b0000100) >> 2
    carrSoln = (flags & 0b0011000 ) >> 3

    stdSay(gnssFixOK == 1, "gnssFixOK==1", "gnssFixOK not 1")
    stdSay(diffSoln == 1, "diffSoln==1", "diffSoln not 1")
    stdSay(relPosValid == 1, "relPosValid==1", "relPosValid not 1")
    stdSay(carrSoln == 2, "carrSoln==2", "carrSoln not 2")


if __name__ == '__main__':
    rospy.loginfo("test_rover is starting")

    # start node
    node = rospy.init_node("test_rover")
    start_time = rospy.get_rostime()

    # set up needed subscriptions    
    sub_ticks = rospy.Subscriber('/rover_ll/ticks', Ticks, ticks_callback, queue_size=10)
    sub_navrelposned = rospy.Subscriber('/ublox/navrelposned', NavRELPOSNED9, navrelposned_callback, queue_size=10)

    # start testing
    waitUntilSomeSecondsIn(3)

    testTicks()    
    testNavrelposned()

