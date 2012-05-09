#!/usr/bin/python

PKG = 'bag_tools' # this package name

import roslib; roslib.load_manifest(PKG)
import rospy
import rosbag
import os
import sys
from optparse import OptionParser

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
  parser = OptionParser(usage="%prog INBAG OUTBAG FRAME_ID TOPICS",
                       description='Create a new bagfile from an existing one replacing the frame id of requested topics.')
  (options, args) = parser.parse_args()
  if len(args) < 4:
    parser.error('Wrong number of arguments')
  inbag = args[0]
  outbag = args[1]
  frame_id = args[2]
  topics = []
  for i in range(len(args) - 3):
    topics.append(args[3+i])
  try:
    change_frame_id(inbag,outbag,frame_id,topics)
  except Exception, e:
    import traceback
    traceback.print_exc()
