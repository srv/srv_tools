#!/usr/bin/python

PKG = 'bag_tools' # this package name

import roslib; roslib.load_manifest(PKG)
import rospy
import rosbag
import os
import sys
import argparse

def change_frame_id(inbag,outbag,frame_id,topics):
  print '   Processing input bagfile: ', inbag
  print '  Writing to output bagfile: ', outbag
  print '            Changing topics: ', topics
  print '           Writing frame_id: ', frame_id

  outbag = rosbag.Bag(outbag,'w')
  for topic, msg, t in rosbag.Bag(inbag,'r').read_messages():
    if topic in topics:
      if msg._has_header:
        msg.header.frame_id = frame_id
    outbag.write(topic, msg, t)
  print 'Closing output bagfile and exit...'
  outbag.close();

if __name__ == "__main__":

  parser = argparse.ArgumentParser(
      description='reate a new bagfile from an existing one replacing the frame id of requested topics.')
  parser.add_argument('-o', metavar='OUTPUT_BAGFILE', required=True, help='output bagfile')
  parser.add_argument('-i', metavar='INPUT_BAGFILE', required=True, help='input bagfile')
  parser.add_argument('-f', metavar='FRAME_ID', required=True, help='desired frame_id name in the topics')
  parser.add_argument('-t', metavar='TOPIC', required=True, help='topic(s) to change', nargs='+')
  args = parser.parse_args()

  try:
    change_frame_id(args.i,args.o,args.f,args.t)
  except Exception, e:
    import traceback
    traceback.print_exc()
