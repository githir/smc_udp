#!/usr/bin/env python
# -*- coding: utf-8 -*-

import socket, threading, time
from contextlib import closing
import rospy

from  autoware_msgs.msg import Lane, Waypoint
import struct

from ctypes import *

#class DesiredCommand(Structure):
#class DesiredCommand(LittleEndianStructure):

class WAYPOINT(BigEndianStructure):
    _pack_ = 1
    _fields_ = [
        ('x',     c_uint32, 32 ),
        ('y',     c_uint32, 32 ),
        ('vel', c_uint16, 16 )
     ]

class RecvWP(BigEndianStructure):
    _fields_ = [
        ('ID',             c_uint8 ),
        ('RollingCounter', c_uint8 ),
        ('CheckSum',       c_uint8 ),
        ('wpcount',        c_uint8),
        ('waypoints',      WAYPOINT * 50),
     ]

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
  pub = rospy.Publisher('MABX_target_waypoints', Lane, queue_size=1) 
  while not rospy.is_shutdown():
    pub.publish(wpmsg)
    r.sleep()

udpcount=0

wpmsg = Lane()
rospy.init_node('mabx_wp_receiver', anonymous=True) 
thread = threading.Thread(target=publisher)
thread.start()

host = rospy.get_param("~udp_recv_hostname", '127.0.0.1')
#host = rospy.get_param("~udp_recv_hostname", '192.168.50.2')
port = int( rospy.get_param("~udp_recv_port", '51001') )
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
rospy.loginfo("starting MABX-WP UDP receiver. hostname:%s, port:%d", host, port)
bufsize=4096
with closing(sock):
  sock.bind((host, port))
  while True:
    data = sock.recv(bufsize)
    udpcount += 1
    fid = map( lambda x: struct.unpack('>B',x)[0], data)[0]
    print map( lambda x: struct.unpack('>B',x)[0], data)
    rospy.loginfo( map( lambda x: struct.unpack('>B',x)[0], data) )

    if fid == 0x04:
      buf = RecvWP.from_buffer_copy(data)
      rospy.loginfo("FrameID %x(DesiredCommand) received.", fid)
      rospy.loginfo("ID %d", buf.ID)
      rospy.loginfo("RollingCounter %d", buf.RollingCounter)
      rospy.loginfo("CheckSum %d", buf.CheckSum)
      rospy.loginfo("wpcount %d", buf.wpcount)
      rospy.loginfo("wp[0].x %0.0f", calc_bin2actual(buf.waypoints[0].x, 0.01, -21474836.48) )
      rospy.loginfo("wp[0].y %0.0f", calc_bin2actual(buf.waypoints[0].y, 0.01, -21474836.48) )
      rospy.loginfo("wp[0].vel %0.0f", calc_bin2actual(buf.waypoints[0].vel, 1./128, 0) )

      wpmsg.header.stamp = rospy.get_rostime()

      waypoints = []
      for i in range(0,buf.wpcount):
        wp = Waypoint()
	wp.pose.pose.position.x = calc_actual2bin(buf.waypoints[i].x, 0.01, -21474836.48)
	wp.pose.pose.position.y = calc_actual2bin(buf.waypoints[i].x, 0.01, -21474836.48)
	wp.twist.twist.linear.x = calc_actual2bin(buf.waypoints[i].vel, 1./128, 0) / 3.6  # m/s
        waypoints.append(wp)

      print waypoints
      wpmsg.waypoints = waypoints

    else:
      rospy.loginfo("FrameID invalid: %2x", fid)

exit(0)


# ctypes --- Pythonのための外部関数ライブラリ
# https://docs.python.org/ja/2.7/library/ctypes.html
# pythonでsocket通信を勉強しよう
# https://qiita.com/__init__/items/5c89fa5b37b8c5ed32a4
# PythonでバイナリをあつかうためのTips
# https://qiita.com/pashango2/items/5075cb2d9248c7d3b5d4

