#!/usr/bin/env python
# -*- coding: utf-8 -*-

import socket  #, time
from contextlib import closing
import rospy
#from  std_msgs.msg import UInt16
from  autoware_msgs.msg import CanInfo 
import struct

from ctypes import *

class TargetStatus(Structure):
#class Indata(LittleEndianStructure):
#class Indata(BigEndianStructure):
    _fields_ = (
        ('FrameID',   c_uint16),
        ('LoopCount', c_uint16),
        ('Mode',      c_uint16),
        ('Speed',     c_uint16),
        ('SteerAngle', c_uint16),
        ('Shift',     c_uint16),
        ('Flasher',   c_uint16),
        ('checksum',  c_uint16)
     )

class ActualStatus(Structure):
    _fields_ = (
        ('FrameID',    c_uint16),
        ('LoopCount',  c_uint16),
        ('Mode',       c_uint16),
        ('Speed',      c_uint16),
        ('SteerAngle',  c_uint16),
        ('Shift',      c_uint16),
        ('Flasher',    c_uint16),
        ('Emergency',  c_uint16),
        ('ThrottlePedal', c_uint16),
        ('BrakePedal', c_uint16),
        ('Fuel',       c_uint16),
        ('checksum',   c_uint16)
     )

def calc_checksum(d):
  return 0xffff  #dummy
  
def calc_actual2bin(val, factor=1, offset=0):
  #print int( (val-offset)/factor )
  return int( (val-offset)/factor )
 

rospy.init_node('smc_receiver', anonymous=True) 
rospy.Publisher('smc_stat', CanInfo, 1) 

host = rospy.get_param("udp_send_hostname", '10.130.3.132')
port = int( rospy.get_param("udp_send_port", '51001') )
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

rospy.loginfo("starting UDP receiver. hostname:%s, port:%d", host, port)

bufsize=4096

with closing(sock):
  sock.bind((host, port))
  while True:
    data = sock.recv(bufsize)
    fid = map( lambda x: struct.unpack('>B',x)[0], data)[0]
    if fid == 0x02:
      buf_target = TargetStatus.from_buffer_copy(data)
      rospy.loginfo("FrameID %x(TargetStatus) received.", fid)
      print "FrameID", buf_target.FrameID
      print "LoopCount",buf_target.LoopCount
      print "Mode",buf_target.Mode
      print "Speed",buf_target.Speed
      print "SteerAngle",buf_target.SteerAngle
      print "Shift",buf_target.Shift
      print "Flasher",buf_target.Flasher
      print "checksum",buf_target.checksum

    elif fid == 0x03:
      buf_actual = ActualStatus.from_buffer_copy(data)
      rospy.loginfo("FrameID %x(ActualStatus) received.", fid)
      print "FrameID", buf_actual.FrameID
      print "LoopCount",buf_actual.LoopCount
      print "Mode",buf_actual.Mode
      print "Speed",buf_actual.Speed
      print "SteerAngle",buf_actual.SteerAngle
      print "Shift",buf_actual.Shift
      print "Flasher",buf_actual.Flasher
      print "Emergency",buf_actual.Emergency
      print "ThrottlePedal",buf_actual.ThrottlePedal
      print "BrakePedal",buf_actual.BrakePedal
      print "Fuel",buf_actual.Fuel
      print "checksum",buf_actual.checksum
    else:
      rospy.loginfo("FrameID invalid: %2x", fid)

exit(0)

# ctypes --- Pythonのための外部関数ライブラリ
# https://docs.python.org/ja/2.7/library/ctypes.html
# pythonでsocket通信を勉強しよう
# https://qiita.com/__init__/items/5c89fa5b37b8c5ed32a4

