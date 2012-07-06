#!/usr/bin/python

PKG = 'bag_tools' # this package name

import roslib; roslib.load_manifest(PKG)
import rospy
import rosbag
import os
import sys
import argparse

def fix_bagfile(inbag, outbag, topics, offset):
    print '   Processing input bagfile: ', inbag
    print '  Writing to output bagfile: ', outbag
    print '            Changing topics: ', topics
    print 'Changing publishing time by: ', offset

    outbag = rosbag.Bag(outbag, 'w')

    time_offset = rospy.Duration.from_sec(offset)

    count = {}
    for topic in topics:
      count[topic] = 0

    for topic, msg, t in rosbag.Bag(inbag).read_messages():
        if topic in topics:
            outbag.write(topic, msg, t + time_offset)
            count[topic] = count[topic] + 1
        else:
            outbag.write(topic, msg, t)
    print 'Closing output bagfile.'
    outbag.close()
    print 'Changed the following:'
    for k, v in count.iteritems():
      print k, ': ', v, ' messages.'

if __name__ == "__main__":

  parser = argparse.ArgumentParser(
      description='Shift the publishing time of given topics in input bagfile.')
  parser.add_argument('-o', metavar='OUTPUT_BAGFILE', required=True, help='output bagfile')
  parser.add_argument('-i', metavar='INPUT_BAGFILE', required=True, help='input bagfile(s)', nargs='+')
  parser.add_argument('-of', metavar='OFFSET', required=True, type=float, help='time offset to add in seconds')
  parser.add_argument('-t', metavar='TOPIC', required=True, help='topic(s) to change', nargs='+')
  args = parser.parse_args()
  try:
      fix_bagfile(args.i, args.o, arg.t, args.of)
  except Exception, e:
      import traceback
      traceback.print_exc()
