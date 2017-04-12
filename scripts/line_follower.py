#!/usr/bin/env python

import roslib; roslib.load_manifest('line_following')
import rospy
import random
import servotest as vc
import math
import numpy as np

#from geometry_msgs.msg import Twist
#from kobuki_msgs.msg import SensorState
from std_msgs.msg import Float64, Int32, Int32MultiArray
from joy_test.msg import IntList
from blobfinder.msg import MultiBlobInfo
from sensor_msgs.msg import LaserScan

# control at 10Hz
CONTROL_PERIOD = rospy.Duration(0.1)

# react to bumpers for 1 second
BUMPER_DURATION = rospy.Duration(1.0)

# search duration of 1 second
SEARCH_DURATION = rospy.Duration(1.5)

# go into line_wait of 1 second
WAIT_DURATION = rospy.Duration(1.0)

# random actions should last 0.5 second
RANDOM_ACTION_DURATION = rospy.Duration(0.5)

#global controller variables
MOTOR_NEUTRAL = 1500    # neutral is 0 from controller to +/-90
ESC_SERVO = 1
STEER_SERVO = 0
STEER_NEUTRAL = 90

# how to write to the controller
#self.controller.setAngle(STEER_SERVO, steering)
#self.controller.setPosition(ESC_SERVO, MOTOR_NEUTRAL + 2*throttle)


# our controller class
class Controller:

    # called when an object of type Controller is created
    def __init__(self):

        # initialize rospy
        rospy.init_node('line_follower')
        self.controller = vc.ServoController()
        
        # set up publisher for commanded velocity
        self.throttle = rospy.Publisher('/mobile_base/commands/throttle',
                                           Int32)
        
        # start out in wandering state
        self.state = 'start'

        # set up a killswitch
        self.kill = 0

        # turning cue
        self.turn = 0
        self.avoid = 0
        self.avoid_angle = 0

        # subscrib to laserscan messages
        rospy.Subscriber('scan', LaserScan, self.laser_callback)

        # pick out a random action to do when we start driving
        #self.start_straight()

        # subscribe to the joystick for the killswitch
        rospy.Subscriber('odroid/commands/combined',
                         IntList, self.motor_callback)
        
        # subscribe to blobfinder messages
        rospy.Subscriber('blobfinder/blue_tape/blobs',
                            MultiBlobInfo, self.blob_callback)

        # set up control timer at 100 Hz
        rospy.Timer(CONTROL_PERIOD, self.control_callback)

############################################################
###########################################################
    # "called when joystick messages come in"
    # from the blobtest.py script

    def motor_callback(self, code):
        button = code.button
        steering = code.steer # range from 0 to 180, 90 mid
        throttle = code.thr # range from -90 to 90, 0 mid
        if button==1:
            print "killswitch engaged; shutting down script, button is:", button
            self.controller.setAngle(STEER_SERVO, 90)
            self.controller.setPosition(ESC_SERVO, MOTOR_NEUTRAL
                                        + 0*throttle)
            self.state = 'walkout'
            self.kill = 1
        #else:
            #self.controller.setAngle(STEER_SERVO, steering)
            #self.controller.setPosition(ESC_SERVO, MOTOR_NEUTRAL + 2*throttle)

    def laser_callback(self, scan):
        ranges = scan.ranges
        max_range = np.nanmax(ranges)
        min_range = np.nanmin(ranges)
        index_min = np.nanargmin(ranges)
        angle_min = scan.angle_min
        angle_max = scan.angle_max
        angle_inc = scan.angle_increment
        #rospy.loginfo('max range is: %f', max_range)
        rospy.loginfo('min range is: %f', min_range)
        # simple avoidance won't work because blob callback is too frequent
        # need to prioritize avoidance
        if (min_range < 0.5):
            self.avoid = 1
            angle = math.degrees(angle_min + (angle_inc * index_min))
            # 28 to -28, 0 mid
            angle = int((angle*3)+90) # convert to 180 to 0 scale, 90 mid
            self.avoid_angle = angle
        else:
            self.avoid = 0
            
    def blob_callback(self, data):
        num = len(data.blobs)
        rospy.loginfo('got a message with %d blobs', num)
        maxes = []
        max_y = []
        numBlob = 0
        screen_width = 480
        for i in range(num):
            '''
            rospy.loginfo('  blob with area %f at (%f, %f)', 
                          data.blobs[i].area,
                          data.blobs[i].cx,
                          data.blobs[i].cy)
            '''
            # blob data reported here.
            # take one with largest area
            maxes.append(data.blobs[i].area)
            maxBlob = max(maxes)
            max_y.append(data.blobs[i].cy)
            maxY = max(max_y)

        # when no blobs appear, enter search state        
        if (num == 0) or (maxBlob < 60) or (maxY < int(screen_width/3)):
            self.state = 'search'
            rospy.loginfo('searching')
        elif (self.avoid == 1):
            self.state = 'avoid'
            rospy.loginfo('set avoid state')
            #self.controller.setAngle(STEER_SERVO, self.avoid_angle) 
        else:
            self.state = 'following'
            # with maxblob, now calculate direction
            numBlob = maxes.index(max(maxes))
            screen_width = 640
            steer_range = 180
            #kp = 0.01
            #theta = kp*(320 - data.blobs[numBlob].cx)
            blob_ratio = data.blobs[numBlob].cx/screen_width
            steering = blob_ratio*steer_range
            steering = steer_range - steering
            steering = int(math.ceil(steering))
            if abs(steering - STEER_NEUTRAL) > 25:
                self.turn = 1
            self.controller.setAngle(STEER_SERVO, steering) 
            rospy.loginfo('blob number: %d',numBlob)
            rospy.loginfo('blob x coordinate: %d',data.blobs[numBlob].cx)
            rospy.loginfo('turn at rate: %d', steering)
            
    # called for 2 seconds when blobs not found
    def look_for_line(self, timer_event = None):
        rospy.loginfo('look_for_line runs')
        self.state = 'search'
        rospy.loginfo('set search to true')
        #rospy.Timer(SEARCH_DURATION, self.look_for_line, oneshot=True)
    
    # called 10 times per second
    def control_callback(self, timer_event=None):
        if self.kill == 1:
            rospy.loginfo('state: kill')
            self.controller.setAngle(STEER_SERVO, STEER_NEUTRAL)
            self.controller.setPosition(ESC_SERVO, MOTOR_NEUTRAL)
            rospy.loginfo('killswitch is engaged. shutting down')
            rospy.signal_shutdown("Killswitch")

        if self.state == 'start':
            rospy.loginfo('state: start')
            thr = 40
            self.controller.setAngle(STEER_SERVO, STEER_NEUTRAL)
            self.controller.setPosition(ESC_SERVO, MOTOR_NEUTRAL +
                                        int(math.ceil(1.5*thr)))
        elif self.state == 'search':
            rospy.loginfo('control search')
            search_angle = 0
            search_thr = 50
            self.controller.setAngle(STEER_SERVO, search_angle)
            self.controller.setPosition(ESC_SERVO, MOTOR_NEUTRAL
                                        + int(math.ceil(1.5*search_thr)))
        elif self.state == 'following':
            rospy.loginfo('state: following')
            if self.turn == 1:
                thr = 50
            else:
                thr = 45
            self.controller.setPosition(ESC_SERVO, MOTOR_NEUTRAL +
                                        int(math.ceil(1.5*thr)))
            rospy.loginfo('throttle is: %d',thr)
        elif self.state == 'avoid':
            rospy.loginfo('avoidance activated')
            thr = 50
            self.controller.setAngle(STEER_SERVO, STEER_NEUTRAL)
            self.controller.setPosition(ESC_SERVO, MOTOR_NEUTRAL)
                                        #+ int(math.ceil(1.5*thr)))
        
    # called by main function below (after init)
    def run(self):
        # timers and callbacks are already set up, so just spin.
        # if spin returns we were interrupted by Ctrl+C or shutdown
        rospy.spin()

# main function
if __name__ == '__main__':
    try:
        ctrl = Controller()
        ctrl.run()
    except rospy.is_shutdown():
        pass
    
