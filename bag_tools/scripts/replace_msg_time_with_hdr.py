#!/usr/bin/python

PKG = 'bag_tools' # this package name

import roslib; roslib.load_manifest(PKG)
import rospy
import rosbag
import sensor_msgs.msg
import os
import sys

def rpl_msg_time_with_hdr(inbag,outbag,in_topic):
  for topic, msg, t in rosbag.Bag(inbag).read_messages():
    # This also replaces tf timestamps under the assumption 
    # that all transforms in the message share the same timestamp
    if topic == in_topic and msg.transforms:
      outbag.write(topic, msg, msg.transforms[0].header.stamp)
    else:
      outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)

if __name__ == "__main__":
  parser = OptionParser(usage="%prog INBAG OUTBAG TOPIC",
                       description='Create a new bagfile from an existing one replacing the message time for the header time.')
  (options, args) = parser.parse_args()
  if len(args) != 3:
    parser.error('Wrong number of arguments')
    inbag = args[0]
    outbag = args[1]
    topic = args[2]
    try:
      rpl_msg_time_with_hdr(inbag, outbag,topic)
    except Exception, e:
      import traceback
      traceback.print_exc()
