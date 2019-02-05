#!/usr/bin/env python
# -*- coding: utf-8 -*-

import socket, struct
import rospy
from  autoware_msgs.msg import VehicleCmd 

from ctypes import *

class DesiredCommand(Structure):
#class Indata(LittleEndianStructure):
#class Indata(BigEndianStructure):
    _fields_ = (
        ('FrameID',   c_uint16),
        ('LoopCount', c_uint16),
        ('Mode',      c_uint16),
        ('Speed',  c_uint16),
        ('SteerAngle',  c_uint16),
        ('Shift', c_uint16),
        ('Flasher',   c_uint16),
        ('checksum', c_uint16)
     )

def udp_send():
  msg =  string_at(pointer(dat),sizeof(dat))
  sock.sendto(msg, (host, port))

def calc_checksum(d):
  d.checksum=0
  msg =  string_at(pointer(d),sizeof(d))
  cs = sum(map(lambda x: int(struct.unpack('B',x)[0]), list(msg))) & 0xff 
#  print map(lambda x: int(struct.unpack('B',x)[0]), list(msg)) , cs
  return cs

def calc_actual2bin(val, factor=1, offset=0):
  return int( (val-offset)/factor )

def cmd_cb(msg):
#  dat.FrameID    = 0x01
  dat.FrameID    = 0x02
  dat.LoopCount += 1
  dat.Mode       = msg.mode
  dat.Speed      = calc_actual2bin(msg.twist_cmd.twist.linear.x, 1/128., 0)
  dat.SteerAngle = msg.steer_cmd.steer #not determine yet
  dat.Shift      = msg.gear
  dat.Flasher    = (msg.lamp_cmd.l<<1) + msg.lamp_cmd.r
  dat.checksum   = calc_checksum(dat)

  udp_send()

rospy.init_node('smc_sender', anonymous=True) 
rospy.Subscriber('smc_cmd', VehicleCmd, cmd_cb) 

dat = DesiredCommand()
host = rospy.get_param("udp_send_hostname", '10.130.3.132')
port = int( rospy.get_param("udp_send_port", '51001') )
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

rospy.loginfo("starting UDP sender. hostname:%s, port:%d", host, port)

rospy.spin()


# ctypes --- Pythonのための外部関数ライブラリ
# https://docs.python.org/ja/2.7/library/ctypes.html
# pythonでsocket通信を勉強しよう
# https://qiita.com/__init__/items/5c89fa5b37b8c5ed32a4
