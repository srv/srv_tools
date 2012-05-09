#!/usr/bin/python

PKG = 'bag_tools' # this package name

import roslib; roslib.load_manifest(PKG)
import rospy
import rosbag
import os
import sys
from optparse import OptionParser

def rpl_msg_time_with_hdr(inbag,outbag):
  print 'Processing input bagfile : ', inbag
  print 'Writing to output bagfile : ', outbag
  outbag = rosbag.Bag(outbag,'w')
  for topic, msg, t in rosbag.Bag(inbag,'r').read_messages():
    # This also replaces tf timestamps under the assumption 
    # that all transforms in the message share the same timestamp
    if topic == "/tf" and msg.transforms:
      outbag.write(topic, msg, msg.transforms[0].header.stamp)
    else:
      outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)
  print 'Closing output bagfile and exit...'
  outbag.close();

if __name__ == "__main__":
  parser = OptionParser(usage="%prog INBAG OUTBAG",
                       description='Create a new bagfile from an existing one replacing the message time for the header time.')
  (options, args) = parser.parse_args()
  if len(args) != 2:
    parser.error('Wrong number of arguments')
  inbag = args[0]
  outbag = args[1]
  try:
    rpl_msg_time_with_hdr(inbag, outbag)
  except Exception, e:
    import traceback
    traceback.print_exc()
