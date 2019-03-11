#!/usr/bin/env python
# -*- coding: utf-8 -*-

import socket  #, time
import time
import threading
from contextlib import closing
import rospy
#from  std_msgs.msg import UInt16
from  autoware_can_msgs.msg import CANInfo 
import struct

from ctypes import *

#class DesiredCommand(Structure):
#class DesiredCommand(LittleEndianStructure):
class DesiredCommand(BigEndianStructure):
    _fields_ = (
        ('ID',             c_uint8 ), 
        ('RollingCounter', c_uint8 ), 
        ('CheckSum',       c_uint8 ), 
        ('Mode',           c_uint8 ), 
        ('Speed',          c_uint16),
        ('SteerAngle',     c_uint16),
        ('Shift',          c_uint8 ),
        ('Flasher',        c_uint8 )
     )

#class TargetStatus(Structure):
class TargetStatus(BigEndianStructure):
    _fields_ = (
        ('ID',             c_uint8 ), 
        ('RollingCounter', c_uint8 ), 
        ('CheckSum',       c_uint8 ), 
        ('Mode',           c_uint8 ), 
        ('Speed',          c_uint16),
        ('SteerAngle',     c_uint16),
        ('Shift',          c_uint8 ),
        ('Flasher',        c_uint8 )
     )

#class ActualStatus(Structure):
class ActualStatus(BigEndianStructure):
    _fields_ = (
        ('ID',             c_uint8 ), 
        ('RollingCounter', c_uint8 ), 
        ('CheckSum',       c_uint8 ), 
        ('Mode',           c_uint8 ), 
        ('Speed',          c_uint16),
        ('SteerAngle',     c_uint16),
        ('Shift',          c_uint8 ),
        ('Flasher',        c_uint8 ),
        ('Emergency',      c_uint8 ),
        ('ThrottlePedal',  c_uint8 ),
        ('BrakePedal',     c_uint8 ),
        ('Fuel',           c_uint8 )
     )

def calc_checksum(d):
  return 0xffff  #dummy
  
def calc_actual2bin(val, factor=1, offset=0):
  #print int( (val-offset)/factor )
  return int( (val-offset)/factor )
 
def calc_bin2actual(val, factor=1, offset=0):
  #print int( (val-offset)/factor )
#  print  val*factor + offset
  return val*factor + offset

def publisher():
  r = rospy.Rate(10)
  pub = rospy.Publisher('smc_stat', CANInfo, queue_size=1) 
  while not rospy.is_shutdown():
#    print "publishing"
#    try:
#      print "udpcount =",udpcount
#    except Exception as e:
#      print "udp not comming yet.",e
    pub.publish(canmsg)
    r.sleep()

udpcount=0

canmsg = CANInfo()
rospy.init_node('smc_receiver', anonymous=True) 
thread = threading.Thread(target=publisher)
thread.start()

host = rospy.get_param("~udp_recv_hostname", '127.0.0.1')
#host = rospy.get_param("~udp_recv_hostname", '192.168.50.2')
port = int( rospy.get_param("~udp_recv_port", '51001') )
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
rospy.loginfo("starting UDP receiver. hostname:%s, port:%d", host, port)
bufsize=4096
with closing(sock):
  sock.bind((host, port))
  while True:
    data = sock.recv(bufsize)
    udpcount += 1
    fid = map( lambda x: struct.unpack('>B',x)[0], data)[0]
#    print map( lambda x: struct.unpack('>B',x)[0], data)
    rospy.loginfo( map( lambda x: struct.unpack('>B',x)[0], data) )

    if fid == 0x01:
      buf = DesiredCommand.from_buffer_copy(data)
      rospy.loginfo("FrameID %x(DesiredCommand) received.", fid)
      rospy.loginfo("ID %d", buf.ID)
      rospy.loginfo("RollingCounter %d", buf.RollingCounter)
      rospy.loginfo("CheckSum %d", buf.CheckSum)
      rospy.loginfo("Mode %d", buf.Mode)
      rospy.loginfo("Speed %0.0f", calc_bin2actual(buf.Speed, 1.0/128, 0))
      rospy.loginfo("SteerAngle %0.0f", calc_bin2actual(buf.SteerAngle, 0.1, -3276.8))
      rospy.loginfo("Shift %d", buf.Shift)
      rospy.loginfo("Flasher %d", buf.Flasher)

    elif fid == 0x02:
      buf = TargetStatus.from_buffer_copy(data)
      rospy.loginfo("FrameID %x(TargetStatus) received.", fid)
      rospy.loginfo("ID %d", buf.ID)
      rospy.loginfo("RollingCounter %d", buf.RollingCounter)
      rospy.loginfo("CheckSum %d", buf.CheckSum)
      rospy.loginfo("Mode %d", buf.Mode)
      rospy.loginfo("Speed %0.0f", calc_bin2actual(buf.Speed, 1.0/128, 0))
      rospy.loginfo("SteerAngle %0.0f", calc_bin2actual(buf.SteerAngle, 0.1, -3276.8))
      rospy.loginfo("Shift %d", buf.Shift)
      rospy.loginfo("Flasher %d", buf.Flasher)

    #  canmsg.header.seq = udpcount    # not work
      canmsg.door = udpcount           # not very good (--;
      rospy.logdebug( "udpcount %d", udpcount )

      canmsg.drvmode = buf.Mode   #need to chek valname'drvmode'
      canmsg.targetveloc = calc_bin2actual(buf.Speed, 1.0/128, 0)  #unit is..  m/s | km/h  ??  need to check
      canmsg.targetangle = calc_bin2actual(buf.SteerAngle, 0.1, -3276.8)
      canmsg.targetshift = buf.Shift #need to check values
      canmsg.light = buf.Flasher #need to check values

    elif fid == 0x03:
      buf = ActualStatus.from_buffer_copy(data)
      rospy.loginfo("FrameID %x(ActualStatus) received.", fid)
      rospy.loginfo("ID %d", buf.ID)
      rospy.loginfo("RollingCounter %d", buf.RollingCounter)
      rospy.loginfo("CheckSum %d", buf.CheckSum)
      rospy.loginfo("Mode %d", buf.Mode)
      rospy.loginfo("Speed %0.0f", calc_bin2actual(buf.Speed, 1.0/128, 0))
      rospy.loginfo("SteerAngle %0.0f", calc_bin2actual(buf.SteerAngle, 0.1, -3276.8))
      rospy.loginfo("Shift %d", buf.Shift)
      rospy.loginfo("Flasher %d", buf.Flasher)
      rospy.loginfo("Emergency %d", buf.Emergency)
      rospy.loginfo("ThrottlePedal %d", buf.ThrottlePedal)
      rospy.loginfo("BrakePedal %d", buf.BrakePedal)
      rospy.loginfo("Fuel %d", buf.Fuel)

    #  canmsg.header.seq = udpcount    # not work
      canmsg.door = udpcount           # not very good solution (--;
      rospy.logdebug( "udpcount %d", udpcount )

      canmsg.drvmode = buf.Mode   #need to chek valname'drvmode'
      canmsg.speed = calc_bin2actual(buf.Speed, 1.0/128, 0)
      canmsg.angle = calc_bin2actual(buf.SteerAngle, 0.1, -3276.8)
   #   canmsg.driveshift = buf.Shift #need to check values
      canmsg.inputshift = buf.Shift #need to check values
      canmsg.light = buf.Flasher #need to check values
   #   canmsg.emeagency = 0
      canmsg.bbrakepress = buf.BrakePedal
      canmsg.brakepedal = buf.BrakePedal  #need to check this.
      canmsg.gaslevel = buf.Fuel

    else:
      rospy.loginfo("FrameID invalid: %2x", fid)

exit(0)

# ctypes --- Pythonのための外部関数ライブラリ
# https://docs.python.org/ja/2.7/library/ctypes.html
# pythonでsocket通信を勉強しよう
# https://qiita.com/__init__/items/5c89fa5b37b8c5ed32a4

