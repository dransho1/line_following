#import serial
import pygame
import time

#serialPort = '/dev/tty.usbserial-A7004Jg4' # Arduino Mega
#serialPort = '/dev/tty.usbmodemfd121'       # Arduino Uno
#baudRate = 9600

# Open Serial Connection to Arduino Board
#ser = serial.Serial(serialPort, baudRate, timeout=1);

'''
Gets joystick data and prints it
'''
def hci_init():
    pygame.init()
    j = pygame.joystick.Joystick(0)
    j.init()
    print('Initialized Joystick : {}'.format(j.get_name()))
    return j

def hci_input(j):
    pygame.event.pump()

    # Used to read input from the two joysticks
    # print "number of axes: ", j.get_numaxes()
    # print "axis 4: ", j.get_axis(4)
    alt_throttle = j.get_axis(4) # axis for xbox controller
    steering = j.get_axis(0)
    throttle = j.get_axis(3)        # axis for PS3 controller
    return (steering, alt_throttle)

def hci_button(j):
    pygame.event.pump()
    button = j.get_button(1)
    return button

def main():
    j = hci_init()
    #while True:
        #print(hci_input(j))
