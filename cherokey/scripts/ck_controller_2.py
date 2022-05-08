#!/usr/bin/env python3

# ##################################################################
# Low-level control of Cherokey bot using rosserial
# ##################################################################

import math

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# custom messages used by rosserial to communicate with Arduino on Cherokey
from cherokey_msgs.msg import WheelState

cherokey_pub = None

class Cherokey():
    '''Cherokey node that abstracts the Arduino-controlled motors

    Assumes node created and running already
    Creates listener on /cherokey/command to accept string-style commands.
    Creates listener on /cherokey/cmd_vel to accept twist messages.
    Creates listener on /cherokey/speedspin to accept messages directly from OpenMower

    Attributes
    ----------
    speed : float
        Speed along the X axis in meters per second; positive is
        forward and negative is backward
    spin : float
        Rotation about the pivot point in radians per second; positive
        is clockwise when viewed from above (right spin)

    Methods
    -------
    stop()
        Stop all movement of the Cherokey

    '''
    def __init__(self, name,
                 wheel_diameter=.066, wheel_base=0.14,
                 cal_distance=1.25, cal_time=5, skew=1.02):
        """
        Parameters
        ----------
        name: str
            The node name that will be used for this robot; defaults
            to "cherokey"
        wheel_diameter : float
            The diameter of the wheels in meters
        wheel_base : float
            The distance between the center of the wheels in meters
        cal_distance : float
            The distance travelled during a calibration time
            when running at 100% power
        cal_time : float
            Time elapsed in the calibration tst
        skew : float
            how much faster is left wheel than right, for correction, e.g. 1.02
        """
        max_rpm = cal_distance / (wheel_diameter * math.pi) / cal_time * 60
        self._left_max_rpm = max_rpm * (1+(skew-1)/2)
        self._right_max_rpm = max_rpm * (1-(skew-1)/2)
        rospy.loginfo(f"max rpm: {max_rpm}")
        rospy.loginfo(f"max rpm left: {self._left_max_rpm}")
        rospy.loginfo(f"max rpm right: {self._right_max_rpm}")
        self._wheel_diameter = wheel_diameter
        self._wheel_base = wheel_base

        self.speed = 0.0
        self.spin = 0.0

        # create subscribers
        self._command_subscription = rospy.Subscriber('cherokey/command', String, self._command_callback)
        self._cmd_vel_subscription = rospy.Subscriber('cherokey/cmd_vel', Twist, self._cmd_vel_callback)
        self._cmd_vel_subscription = rospy.Subscriber('cherokey/speedspin', String, self._cmd_speedspin)

        # just testing
        self._set_motor_speeds()

    def _command_callback(self, msg):
        command = msg.data
        if command == 'test':
            rospy.loginfo("got test msg")
        elif command == 'stop':
            self.stop()
        else:
            print('Unknown command, stopping instead')
            self.stop()

    def stop(self):
        self.speed = 0
        self.spin = 0
        self._set_motor_speeds()

    def max_speed(self):
        '''Speed in meters per second at maximum RPM'''
        rpm = (self._left_max_rpm + self._right_max_rpm) / 2.0
        mps = rpm * math.pi * self._wheel_diameter / 60.0
        return mps

    def max_twist(self):
        '''Rotation in radians per second at maximum RPM'''
        return self.max_speed() / self._wheel_diameter

    def _cmd_vel_callback(self, msg):
        self.speed = msg.linear.x
        self.spin = msg.angular.z
        self._set_motor_speeds()

    def _cmd_speedspin(self, msg):
        # TODO: use real ROS custom message for communicating speed
        self.speed = float(msg.data.split()[0])
        self.spin = float(msg.data.split()[1])
        # rospy.loginfo(f"Got speedspin topic message {msg.data}")
        # self.speed = 0.0
        # self.spin = 0.0
        self._set_motor_speeds()

    def _set_motor_speeds(self):
        # TODO: inject a stop() if no speeds seen for a while
        #
        # max value that can be written to wheels, corresponding to the set max_rpm values
        signal_range = 255 
        # First figure out the speed of each wheel based on spin: each wheel
        # covers self._wheel_base meters in one radian, so the target speed
        # for each wheel in meters per sec is spin (radians/sec) times
        # wheel_base divided by wheel_diameter
        #
        right_twist_mps = self.spin * self._wheel_base / self._wheel_diameter
        left_twist_mps = -1.0 * self.spin * \
            self._wheel_base / self._wheel_diameter
        #
        # Now add in forward motion.
        #
        left_mps = self.speed + left_twist_mps
        right_mps = self.speed + right_twist_mps
        #
        # Convert meters/sec into RPM: for each revolution, a wheel travels
        # pi * diameter meters, and each minute has 60 seconds.
        #
        left_target_rpm = (left_mps * 60.0) / (math.pi * self._wheel_diameter)
        right_target_rpm = (right_mps * 60.0) / (math.pi * self._wheel_diameter)
        #
        left_percentage = (left_target_rpm / self._left_max_rpm) * 100.0
        right_percentage = (right_target_rpm / self._right_max_rpm) * 100.0
        #
        # clip to +- 100%
        left_percentage = max(min(left_percentage, 100.0), -100.0)
        right_percentage = max(min(right_percentage, 100.0), -100.0)
        #
        left_signal = (left_target_rpm / self._left_max_rpm) * signal_range
        right_signal = (right_target_rpm / self._right_max_rpm) * signal_range
        #
        left_dir = -1 if left_signal < 0 else 1
        right_dir = -1 if right_signal < 0 else 1
        left_signal_abs = left_signal * left_dir
        right_signal_abs = right_signal * right_dir

        # clip to signal max and deliver int
        signal_max =  max(left_signal_abs, right_signal_abs)
        if (signal_max > signal_range):
            left_signal_abs = left_signal_abs / signal_max * signal_range
            right_signal_abs = right_signal_abs / signal_max * signal_range
        left_signal_abs = int(min(left_signal_abs, signal_range))
        right_signal_abs = int(min(right_signal_abs, signal_range))
        #
        # rospy.logdebug(f"left_mps: {left_mps:.3f} right_mps: {right_mps:.3f}")
        # rospy.logdebug(f"left_target_rpm: {left_target_rpm:.3f} right_target_rpm: {right_target_rpm:.3f}")
        # rospy.logdebug(f"left_signal_abs: {left_signal_abs:.3f} right_signal_abs: {right_signal_abs:.3f}")

        pub_wheels.publish(left_signal_abs, left_dir, right_signal_abs, right_dir)        




if __name__ == '__main__':
    rospy.loginfo("cherokey is starting")

    global pub_wheels

    # start node
    node = rospy.init_node("cherokey")
    
    # start publisher for sending data over rosserial to Arduino on Cherokey
    pub_wheels = rospy.Publisher("wheels_set_state", WheelState, queue_size=10)


    # start cherokey 
    cherokey = Cherokey('cherokey', cal_distance=1.25, cal_time=5, skew=1.15)
    rospy.loginfo("cherokey started")

    # register publisher for controlling physical wheels 
    # rotspeed = 155
    # while not rospy.is_shutdown() and rotspeed < 500:
    #     rospy.loginfo(f"setting rot to {rotspeed}")
    #     pub_wheels = rospy.Publisher("wheels_set_state", WheelState, queue_size=10)
    #     pub_wheels.publish(rotspeed,1,rotspeed,1)
    #     rospy.sleep(1.0)
    #     rotspeed = rotspeed + 100
    # pub_wheels.publish(0,1,0,1)


    #while not rospy.is_shutdown():
    # do some basic tests
    cherokey.spin = 0

    cherokey.speed = 0.1 # in meters/sec
    cherokey._set_motor_speeds()
    rospy.sleep(1.0)

    cherokey.speed = 0.1 # in meters/sec
    cherokey._set_motor_speeds()
    rospy.sleep(1.0)

    cherokey.speed = 0.1 # in meters/sec
    cherokey._set_motor_speeds()
    rospy.sleep(1.0)

    # back up one metre...
    cherokey.speed = -0.5 # in meters/sec
    cherokey._set_motor_speeds()
    rospy.sleep(2.0)

    cherokey.speed = 0.0 # in meters/sec
    cherokey._set_motor_speeds()
    rospy.sleep(10.0)

    cherokey.speed = 10.0 # in meters/sec
    cherokey._set_motor_speeds()
    rospy.sleep(5.0)

    cherokey.speed = 0 # in meters/sec
    cherokey._set_motor_speeds()
    rospy.sleep(1.0)


    rospy.spin()

    rospy.loginfo("cherokey is ending")

