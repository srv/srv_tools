#!/usr/bin/python

PKG = 'bag_tools' # this package name

import roslib; roslib.load_manifest(PKG)
import rospy
import rosbag
import os
import sys
import argparse

def cut(inbags, outbagfile, start, duration):
  start_time = rospy.Time.from_sec(999999999999)
  for inbag in inbags:
    print '   Looking for smallest time in:', inbag
    for topic, msg, t in rosbag.Bag(inbag,'r').read_messages():
      if t < start_time:
        start_time = t
      break
  print '   Bagfiles start at', start_time
  start_time = start_time + rospy.Duration.from_sec(start)
  end_time = start_time + rospy.Duration.from_sec(duration)
  print '   Cutting out from', start_time, 'to', end_time
  outbag = rosbag.Bag(outbagfile, 'w')
  num_messages = 0
  for inbag in inbags:
    print '   Extracting messages from:', inbag
    for topic, msg, t in rosbag.Bag(inbag,'r').read_messages(start_time=start_time, end_time=end_time):
      outbag.write(topic, msg, t)
      num_messages = num_messages + 1
  outbag.close()
  print'    New output bagfile has', num_messages, 'messages'


if __name__ == "__main__":
  parser = argparse.ArgumentParser(
      description='Cuts out a section from an input bagfile and writes it to an output bagfile')
  parser.add_argument('--inbag', help='input bagfile(s)', nargs='+', required=True)
  parser.add_argument('--outbag', help='output bagfile', required=True)
  parser.add_argument('--start', help='start time', type=float, required=True)
  parser.add_argument('--duration', help='duration of the resulting part', type=float, required=True)
  args = parser.parse_args()
  try:
    cut(args.inbag, args.outbag, args.start, args.duration)
  except Exception, e:
    import traceback
    traceback.print_exc()
