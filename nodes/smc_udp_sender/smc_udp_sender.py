#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math, socket, struct
import rospy
from  autoware_msgs.msg import VehicleCmd 

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

def udp_send():
  msg =  string_at(pointer(dat),sizeof(dat))
  sock.sendto(msg, (host, port))

def calc_checksum(d):
  d.CheckSum=0
  msg =  string_at(pointer(d),sizeof(d))
  cs = 0x100 - sum(map(lambda x: int(struct.unpack('B',x)[0]), list(msg))) & 0xff
  return  cs
  
def calc_actual2bin(val, factor=1, offset=0):
  return int( (val-offset)/factor )

cbcount = 0
def cmd_cb(msg):
  global cbcount
  cbcount += 1
  if(not cbcount % 100):
    rospy.logdebug("/VehicleCmd callback count = %d",cbcount)

  dat.ID              = 0x01
  dat.RollingCounter += 1
  dat.Mode            = msg.mode #Incomplete. Need to be adjusted
  #運転モード 指示値
  # 0:手動
  # 1:自動

  speed = msg.ctrl_cmd.linear_velocity
  angle = msg.ctrl_cmd.steering_angle
  dat.Speed = calc_actual2bin(speed, 1/128., 0) #m/s
  dat.SteerAngle = calc_actual2bin(angle, 0.1, -3276.8)

#  dat.Speed           = calc_actual2bin(msg.twist_cmd.twist.linear.x, 1/128., 0) #m/s
#  dat.Speed           = calc_actual2bin(msg.twist_cmd.twist.linear.x * 3.6, 1/128., 0) #km/h
#  dat.SteerAngle      = msg.steer_cmd.steer #not determine yet # deg
#  dat.SteerAngle      = calc_actual2bin(msg.steer_cmd.steer*180/math.pi, 0.1, -3276.8)  #need to be adjusted #deg
#  print "stter rad:",msg.steer_cmd.steer,"steer deg:",msg.steer_cmd.steer*180/math.pi,"uint16:",calc_actual2bin(msg.steer_cmd.steer*180/math.pi, 0.1, -3276.8)

  dat.Shift           = msg.gear #Incomplete. Need to be adjusted
  #シフト 指示値
  # 0x0:Shift in Progress
  # 0x1:L
  # 0x8:R
  # 0x9:N
  # 0xA:P
  # 0xB:D
  # 0xD:S
  # 0xE:M
  dat.Flasher         = ((msg.lamp_cmd.r & 0x01) <<1) + (msg.lamp_cmd.l & 0x01)
  dat.CheckSum        = calc_checksum(dat)

  udp_send()

rospy.init_node('smc_sender', anonymous=True) 
rospy.Subscriber('smc_cmd', VehicleCmd, cmd_cb) 

dat = DesiredCommand()
host = rospy.get_param("udp_send_hostname", '127.0.0.1')
#host = rospy.get_param("~udp_send_hostname", '192.168.0.1')
port = int( rospy.get_param("~udp_send_port", '30000') )
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

rospy.loginfo("starting UDP sender. hostname:%s, port:%d", host, port)

rospy.spin()


# ctypes --- Pythonのための外部関数ライブラリ
# https://docs.python.org/ja/2.7/library/ctypes.html
# pythonでsocket通信を勉強しよう
# https://qiita.com/__init__/items/5c89fa5b37b8c5ed32a4
