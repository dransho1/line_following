#!/usr/bin/env python

import roslib; roslib.load_manifest('line_following')
import rospy
import random
import servotest as vc
import math

#from geometry_msgs.msg import Twist
#from kobuki_msgs.msg import SensorState
from std_msgs.msg import Float64, Int32, Int32MultiArray
from joy_test.msg import IntList
from blobfinder.msg import MultiBlobInfo

# control at 100Hz
CONTROL_PERIOD = rospy.Duration(0.01)

# react to bumpers for 1 second
BUMPER_DURATION = rospy.Duration(1.0)

# search duration of 1 second
SEARCH_DURATION = rospy.Duration(1.5)

# wait duration of 2 seconds
WAIT_DURATION = rospy.Duration(1.0)

# random actions should last 0.5 second
RANDOM_ACTION_DURATION = rospy.Duration(0.5)

#global controller variables
MOTOR_NEUTRAL = 1500
ESC_SERVO = 1
STEER_SERVO = 0

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
        self.state = 'wander'

        # pick out a random action to do when we start driving
        self.start_straight()

        # we will stop driving if picked up or about to drive off edge
        # of something
        self.cliff_alert = 0

        # subscribe to the joystick for the killswitch
        rospy.Subscriber('odroid/commands/combined',
                         IntList, self.motor_callback)
        
        '''
        # set up subscriber for sensor state for bumpers/cliffs
        rospy.Subscriber('/mobile_base/sensors/core',
                         SensorState, self.sensor_callback)
        '''
        # subscribe to blobfinder messages
        rospy.Subscriber('blobfinder/blue_tape/blobs',
                            MultiBlobInfo, self.blob_callback)

        # set up control timer at 100 Hz
        rospy.Timer(CONTROL_PERIOD, self.control_callback)

        # set up timer for random actions at 2 hz
        #rospy.Timer(RANDOM_ACTION_DURATION, self.pick_random_action)
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
            rospy.signal_shutdown("Killswitch")
    
    def blob_callback(self, data):

        num = len(data.blobs)
        rospy.loginfo('got a message with %d blobs', num)
        maxes = []
        numBlob = 0
        for i in range(num):
            rospy.loginfo('  blob with area %f at (%f, %f)', 
                          data.blobs[i].area,
                          data.blobs[i].cx,
                          data.blobs[i].cy)
            # blob data reported here.
            # take one with largest area
            maxes.append(data.blobs[i].area)
            maxBlob = max(maxes)

        # when no blobs appear, enter look_for_line callback
        # should wait certain duration before calling Timer, and after
        # waiting, then it can check if number of blobs are still zero 

        if (num == 0) or (maxBlob < 40):
            # enter walkout state, callback in 1 second
            self.state = 'walkout'
            rospy.Timer(WAIT_DURATION, self.look_for_line, oneshot=True)
        else:

            # with maxblob, now calculate direction
            numBlob = maxes.index(max(maxes))
            kp = 0.01
            theta = kp*(320 - data.blobs[numBlob].cx)
            #self.wander_action.angular.z = theta
            steering = math.ceil(abs(theta*90))
            self.controller.setAngle(STEER_SERVO, int(steering)) 
            print 'theta is:', theta
            print 'steering is:',int(steering)

            rospy.loginfo('blob number: %d',numBlob)
            rospy.loginfo('blob x coordinate: %d',data.blobs[numBlob].cx)
            rospy.loginfo('turn at rate: %d', theta)
            
    # called when no blobs appear 
    def look_for_line(self, timer_event = None):

        rospy.loginfo('look_for_line called')

        # if there are no blobs, enter search state, reset
        if self.state == 'walkout':
            rospy.loginfo('exit walkout state')
            self.state = 'search'
            # reset in two seconds to finish state
            rospy.Timer(SEARCH_DURATION, self.look_for_line, oneshot=True)

        # if already searching, reset to wander
        else:  
            rospy.loginfo('exit')
            self.state = 'wander'

    '''
    # called whenever sensor messages are received
    def sensor_callback(self, msg):

        # set cliff alert
        self.cliff_alert = msg.cliff

        # ignore bumper if we are already reacting to it
        if self.state in ['backward', 'turn_left', 'turn_right']:
            return

        # see what we should do next
        next_state = None

        if msg.bumper & SensorState.BUMPER_CENTRE:
            next_state = 'backward'
        elif msg.bumper & SensorState.BUMPER_LEFT:
            next_state = 'turn_right'
        elif msg.bumper & SensorState.BUMPER_RIGHT:
            next_state = 'turn_left'

        # if bumped, go to next state
        if next_state is not None:
            
            self.state = next_state

            # in 1 second, finish this state. waits 1 second, then callback
            rospy.Timer(BUMPER_DURATION, self.bumper_done, oneshot=True)


    # called when we are done with a bumper reaction
    def bumper_done(self, timer_event=None):

        # if we just backed up, time to turn
        if self.state == 'backward':
            
            # go to turning state
            self.state = 'turn_left'
            # reset again in a second
            rospy.Timer(BUMPER_DURATION, self.bumper_done, oneshot=True)
                        
        else: # we just turned, so go return to wandering
            
            self.state = 'wander'
    '''
    
    # called when it's time to choose a new random wander direction
    def start_straight(self, timer_event=None):
        # goes fwd
        #self.wander_action = Twist()
        #self.wander_action.linear.x = 0.2
        self.controller.setAngle(STEER_SERVO, 90) # 90 is staying straight
        self.controller.setPosition(ESC_SERVO, MOTOR_NEUTRAL + 2*45) 
        #self.wander_action.angular.z = random.uniform(-1.0, 1.0)
    
    # called 100 times per second
    def control_callback(self, timer_event=None):

        # initialize commanded vel to 0, 0
        #cmd_vel = Twist()

        # only set commanded velocity to non-zero if not picked up:
        if not self.cliff_alert:
            p_thr = 30
            n_thr = -45

            # state maps straightforwardly to command
            if self.state == 'backward':
                #cmd_vel.linear.x = -0.3
                thr = n_thr
                self.controller.setPosition(ESC_SERVO, MOTOR_NEUTRAL + 2*thr)
            elif self.state == 'turn_left':
                #cmd_vel.angular.z = 1.5
                self.controller.setAngle(STEER_SERVO, 45)
            elif self.state == 'turn_right':
                #cmd_vel.angular.z = -1.5
                self.controller.setAngle(STEER_SERVO, 135)
                
            # need to change the search state!!
            elif self.state == 'search':
                #cmd_vel.angular.z = 0.2
                self.controller.setAngle(STEER_SERVO, 45)
            elif self.state == 'walkout':
                #cmd_vel.angular.x = 0.15
                thr = p_thr
                self.controller.setPosition(ESC_SERVO, MOTOR_NEUTRAL + 2*thr)
            else: # forward
                #cmd_vel = self.wander_action
                thr = p_thr + 10
                self.controller.setPosition(ESC_SERVO, MOTOR_NEUTRAL + 2*thr)

        self.throttle.publish(thr)
        
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
    except rospy.ROSInterruptException:
        pass
    
