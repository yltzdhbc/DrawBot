'''
@Author: ylt
@Date: 2019-10-27 16:12:09
@LastEditTime: 2019-11-23 16:08:16
@LastEditors: ylt
@Description: In User Settings Edit
@FilePath: \WebServer\ROBOT_Class.py
'''

import socket
import time
import struct


class ROBOT(object):
    UDP_IP = "192.168.4.1"  # default robot ip
    UDP_PORT = 2222  # defeat robot port
    sock = 0
    sock_rec = 0
    PI = 3.1415926

    def __init__(self):
        # Create default socket with UDP protocol
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_rec = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_rec.bind((self.get_host_ip(), 2223))

    # get_host_ip: get local ip
    def get_host_ip(self):
        try:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.s.connect(('8.8.8.8', 80))
            self.ip = self.s.getsockname()[0]
        finally:
            self.s.close()

        print "ip : ", self.ip
        return self.ip

    # sendCommand: to send messages
    def sendCommand(self, OSCmessage, param):
        base = bytearray(OSCmessage)  # OSC message
        param = bytearray(struct.pack(">f", param))
        message = base + param
        self.sock.sendto(message, (self.UDP_IP, self.UDP_PORT))
        time.sleep(0.05)

    # recMessage: to recrive messages
    def recMessage(self):
        data = self.sock_rec.recv(40)
        data = data.decode()
        time.sleep(0.05)
        print(data)

    # Mode: 0 NORMAL MODE, 1 PRO MODE
    def mode(self, value=0.0):
        print "ROBOT Mode:", value
        if (value != 1):
            value = 0.0
        # Encapsulate the commands on OSC protocol UDP message and send...
        self.sendCommand(b'/1/toggle1/\x00\x00,f\x00\x00', value)

    # Throttle command. Values from [-1.0 to 1.0] positive: forward
    def throttle(self, value=0.0):
        print "ROBOT Throttle:", value
        value = (value + 1.0) / 2.0  # Adapt values to 0.0-1.0 range
        self.sendCommand(b'/1/fader1\x00\x00\x00,f\x00\x00',
                         value)  # send OSC message

    # Steering command. Values from [-1.0 to 1.0] positive: turn right
    def steering(self, value=0.0):
        print "ROBOT Steering:", value
        value = (value + 1.0) / 2.0  # Adapt values to 0.0-1.0 range
        self.sendCommand(b'/1/fader2\x00\x00\x00,f\x00\x00',
                         value)  # send OSC message

    # Servo1 command. Values 0 or 1 (activated)
    def servo1(self, value=0.0):
        print "ROBOT Servo1:", value
        # if (value!=1):value=0.0
        self.sendCommand(b'/1/push1\x00\x00,f\x00\x00',
                         value)  # send OSC message

    # Servo2 command.(value) Values from [-1.0 to 1.0]
    def servo2(self, value=0.0):
        print "ROBOT Servo2:", value
        value = (value + 1.0) / 2.0  # Adapt values to 0.0-1.0 range
        self.sendCommand(b'/1/fader3\x00\x00\x00,f\x00\x00',
                         value)  # send OSC message

    # Move speed, steps1, steps2
    def move(self, speed, steps1, steps2):
        base = bytearray(b'/1/move\x00\x00\x00')
        param1 = bytearray(struct.pack("h", speed))
        param2 = bytearray(struct.pack("h", steps1))
        param3 = bytearray(struct.pack("h", steps2))
        message = base + param1 + param2 + param3
        self.sock.sendto(message, (self.UDP_IP, self.UDP_PORT))
        print "ROBOT MOVE", speed, steps1, steps2

    # line command.(speed, distance) distance + forward   - backward
    def line(self, distance):
        self.recMessage()
        setps = distance * 512 / (51.75 * 3.11412) * 2
        self.move(1, setps, setps)
        print "ROBOT line:", distance

    # turn command.(speed, angle) angle + left    - right
    def turn(self, angle):
        self.recMessage()
        setps = (3400) / 360 * angle
        self.move(1, -setps, setps)
        print "ROBOT turn:", angle

    # # circularArc
    # def circularArc(self, speed, dir, angle, radius):
    #     if dir == 0:
    #         left_setps = angle * self.PI * \
    #             (radius - self.wheel_base/2)/180*11428
    #         right_setps = angle * self.PI * \
    #             (radius + self.wheel_base/2)/180*11428
    #     elif dir == 1:
    #         left_setps = angle * self.PI * \
    #             (radius + self.wheel_base/2)/180*11428
    #         right_setps = angle * self.PI * \
    #             (radius - self.wheel_base/2)/180*11428
    #     self.move(speed, int(left_setps), int(right_setps))
    #     print "ROBOT Arc:", speed, dir, angle, radius

    # # pidAdjust.(pid, value) pid P_Stability D_Stability P_Speed I_Speed
    # # pidAdjust.value -100 ----- +100  percent of pid value
    # def pidAdjust(self, pid, value):
    #     value = (value + 1.0) / 2.0 / 100
    #     if pid == 'P_Stability':
    #         self.sendCommand(b'/2/fader1\x00\x00\x00,f\x00\x00', value)
    #         print "P_Stability:", value, '%'
    #     elif pid == 'D_Stability':
    #         self.sendCommand(b'/2/fader2\x00\x00\x00,f\x00\x00', value)
    #         print "D_Stability:", value, '%'
    #     elif pid == 'P_Speed':
    #         self.sendCommand(b'/2/fader3\x00\x00\x00,f\x00\x00', value)
    #         print "P_Speed:", value, '%'
    #     elif pid == 'I_Speed':
    #         self.sendCommand(b'/2/fader4\x00\x00\x00,f\x00\x00', value)
    #         print "I_Speed:", value, '%'

    # # line command.(speed, distance) distance + forward   - backward
    # def line(self, speed, distance):
    #     self.recMessage()
    #     setps = 1 * distance
    #     self.move(1, setps, setps)
    #     print "BROBOT line:", speed, distance

    # # turn command.(speed, angle) angle + left    - right
    # def turn(self, speed, angle):
    #     setps = (4720 - 40) / 360 * angle
    #     self.move(1, -setps, setps)
    #     print "BROBOT turn:", speed, angle

    # def setp(self, distance):
    #     steps = distance * 512 / (51.75 * 3.1412)
    #     return steps

    # def moveForward(self, distance):
    #     steps = self.step(distance)
    #     self.move(1, setps, setps)
    #     print "MoveForward:",  setps , distance

    # def moveBackward(self, distance):
    #     steps = self.step(distance)
    #     self.move(1, -setps, -setps)
    #     print "moveBackward:",  setps , distance

    # def turnLeft(self, degrees):
    #     rotation = getNearestAngle(degrees) / 360.0
    #     distance = wheel_base * 3.1412 * rotation
    #     steps = self.step(distance)
    #     self.move(1, -setps, setps)
    #     print "turnLeft:",  setps, degrees

    # def turnRight(self, degrees):
    #     rotation = getNearestAngle(degrees) / 360.0
    #     distance = wheel_base * 3.1412 * rotation
    #     steps = self.step(distance)
    #     self.move(1, setps, -setps)
    #     print "turnRight:",  setps, degrees
