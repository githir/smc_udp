#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import math, socket, struct
import rospy
from  autoware_msgs.msg import Lane 
from geometry_msgs.msg import PoseStamped

from ctypes import *

#class DesiredCommand(Structure):
#class DesiredCommand(LittleEndianStructure):
class POSE(BigEndianStructure):
    _fields_ = (
        ('x',     c_uint32 ),
        ('y',     c_uint32 ),
        ('yaw',   c_uint16 )
     )

class WAYPOINT(BigEndianStructure):
    _fields_ = (
        ('x',     c_uint32, 32 ),
        ('y',     c_uint32, 32 ),
        ('speed', c_uint16, 16 )
     )

class SendWP(BigEndianStructure):
    _fields_ = (
        ('ID',             c_uint8 ),
        ('RollingCounter', c_uint8 ),
        ('CheckSum',       c_uint8 ),
        ('selfPose',       POSE ),
        ('wpcount',        c_uint8),
        ('waypoints',      WAYPOINT * 50),
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

cbcount_pose = 0
cbcount_wp = 0
time_poseUpdate = None
time_wpUpdate = None

def pose_cb(msg):
  global cbcount_pose, time_poseUpdate
  cbcount_pose += 1
  time_poseUpdate = rospy.rostime.get_rostime()
  print "###TIME###",time_poseUpdate

  #if(not cbcount_pose % 100):
  if True:
    rospy.logdebug("/current_pose callback count = %d",cbcount_pose)
   
  pos_x = msg.pose.position.x
  pos_y = msg.pose.position.y
  rot_z = msg.pose.orientation.z 
  print '============================================================'
  print pos_x,pos_y,rot_z
  print '============================================================'
  dat.selfPose.x = calc_actual2bin(pos_x, 0.01, -21474836.48)
  dat.selfPose.y = calc_actual2bin(pos_y, 0.01, -21474836.48)
  dat.selfPose.yaw = calc_actual2bin(rot_z/math.pi*180. , 1./128, 0)  # deg

def wp_cb(msg):
  global cbcount_wp, time_wpUpdate
  cbcount_wp += 1
  time_wpUpdate = rospy.rostime.get_rostime()

  #if(not cbcount_wp % 100):
  if True:
    rospy.logdebug("/final_waypoints callback count = %d",cbcount_wp)

  wpcount = 0
  print '-------------------------------------------------------------'
  try:
    while True:
      pos_x = msg.waypoints[wpcount].pose.pose.position.x
      pos_y = msg.waypoints[wpcount].pose.pose.position.y
      pose = msg.waypoints[wpcount].pose.pose.orientation.z
      vel_x = msg.waypoints[wpcount].twist.twist.linear.x
      print pos_x, pos_y, vel_x
      wpcount += 1

      try:
          dat.waypoints[wpcount].x = calc_actual2bin(pos_x, 0.01, -21474836.48)
          dat.waypoints[wpcount].y = calc_actual2bin(pos_y, 0.01, -21474836.48)
          dat.waypoints[wpcount].speed = calc_actual2bin(vel_x*3.6, 1./128, 0)  # km/h
      except Exception as e:
        print "#################################"
        print e
        print "#################################"
        time.sleep(5)

  except Exception as e:
    print e
    print "wpcount =",wpcount
    dat.wpcount = wpcount

  #if 
  print dat

  dat.ID              = 0x04
  dat.RollingCounter += 1

  dat.ID              = 0x04
  dat.RollingCounter = 0xff
  dat.CheckSum = 0xEE
  dat.selfPose.x = 11
  dat.selfPose.y = 22
  dat.selfPose.yaw = 33
  dat.wpcount = 100
  dat.waypoints[0].x = 111
  dat.waypoints[0].y = 112
  dat.waypoints[0].speed = 113
  dat.waypoints[1].x = 211
  dat.waypoints[1].y = 212
  dat.waypoints[1].speed = 213
  dat.waypoints[2].x = 221
  dat.waypoints[2].y = 222
  dat.waypoints[2].speed = 223
 

#   class SendWP(BigEndianStructure):
#     _fields_ = (
#         ('ID',             c_uint8 ),
#         ('RollingCounter', c_uint8 ),
#         ('CheckSum',       c_uint8 ),
#         ('selfPose',       POSE * 1 ),
#         ('wpcount',        c_uint8),
#         ('waypoints',      WAYPOINT * 50)
# 


  udp_send()

rospy.init_node('smc_WPsender', anonymous=True) 
#rospy.Subscriber('smc_cmd', VehicleCmd, cmd_cb) 
rospy.Subscriber('final_waypoints', Lane, wp_cb) 
rospy.Subscriber('current_pose', PoseStamped, pose_cb) 

dat = SendWP()
host = rospy.get_param("~udp_send_hostname", '127.0.0.1')
#host = rospy.get_param("~udp_send_hostname", '192.168.0.1')
port = int( rospy.get_param("~udp_send_port", '30000') )
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

rospy.loginfo("starting UDP sender. hostname:%s, port:%d", host, port)

rospy.spin()


# ctypes --- Pythonのための外部関数ライブラリ
# https://docs.python.org/ja/2.7/library/ctypes.html
# pythonでsocket通信を勉強しよう
# https://qiita.com/__init__/items/5c89fa5b37b8c5ed32a4