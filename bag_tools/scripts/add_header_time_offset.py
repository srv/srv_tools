#!/usr/bin/python

PKG = 'bag_tools' # this package name

import roslib; roslib.load_manifest(PKG)
import rospy
import rosbag
import os
import sys
import argparse

def add_offset(inbags, topics, offset):
  for inbag in inbags:
    outbag = "timefixed-" + inbag
    print '      Processing input bagfile: ', inbag
    print '     Writing to output bagfile: ', outbag
    print 'Changing header time of topics: ', topics
    print '                 Adding offset: ', offset
    outbag = rosbag.Bag(outbag,'w')
    time_offset = rospy.Duration.from_sec(offset)
    for topic, msg, t in rosbag.Bag(inbag,'r').read_messages():
      if topic in topics:
        if topic == "/tf":
          for transform in msg.transforms:
            transform.header.stamp += time_offset
        elif msg._has_header:
          msg.header.stamp += time_offset
      outbag.write(topic, msg, t)
    outbag.close()


if __name__ == "__main__":
  parser = argparse.ArgumentParser(
      description='Changes header timestamps using given offset, can change'
                  '/tf as well.')
  parser.add_argument('-o', metavar='OFFSET', required=True, type=float, help='time offset to add in seconds')
  parser.add_argument('-i', metavar='BAGFILE', required=True, help='input bagfile(s)', nargs='+')
  parser.add_argument('-t', metavar='TOPIC', required=True, help='topics to change', nargs='+')
  args = parser.parse_args()
  try:
    add_offset(args.i, args.t, args.o)
  except Exception, e:
    import traceback
    traceback.print_exc()
