import serial
import time

class ServoController:
    def __init__(self):
        usbPort = '/dev/ttyACM0'
        self.sc = serial.Serial(usbPort, timeout=1)

    def closeServo(self):
        self.sc.close()

    def setAngle(self, n, angle):
        # this code works for sure in python shell
        if angle > 180 or angle <0:
           angle=90
        byteone=int(254*angle/180)
        bud=chr(0xFF)+chr(n)+chr(byteone)
        self.sc.write(bud)

    def setPosition(self, servo, position):
        position = position * 4
        poslo = (position & 0x7f)
        poshi = (position >> 7) & 0x7f
        chan  = servo &0x7f
        data =  chr(0xaa) + chr(0x0c) + chr(0x04) + chr(chan) + chr(poslo) + chr(poshi)
        self.sc.write(data)

    def getPosition(self, servo):
        chan  = servo &0x7f
        data =  chr(0xaa) + chr(0x0c) + chr(0x10) + chr(chan)
        self.sc.write(data)
        w1 = ord(self.sc.read())
        w2 = ord(self.sc.read())
        return w1, w2

    def getErrors(self):
        data =  chr(0xaa) + chr(0x0c) + chr(0x21)
        self.sc.write(data)
        w1 = ord(self.sc.read())
        w2 = ord(self.sc.read())
        return w1, w2

    def setPWM(self, duty_cycle, period, frequency=None):
        if frequency:
            period = 1.0/frequency
        on = int((duty_cycle/100.0)*period)
        on_low = (on & 0x7f)
        on_high = (on >> 7) & 0x7f
        period = int(period)
        period_low = period & 0x7f
        period_high = (period >> 7) & 0x7f
        data = chr(0xaa) + chr(0x0c) + chr(0x0a) + chr(on_low) + chr(on_high) + chr(period_low) + chr(period_high)
        self.sc.write(data)

    def triggerScript(self, subNumber):
        data =  chr(0xaa) + chr(0x0c) + chr(0x27) + chr(0)
        self.sc.write(data)
