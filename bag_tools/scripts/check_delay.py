#!/usr/bin/python

PKG = 'bag_tools' # this package name

import roslib; roslib.load_manifest(PKG)
import rospy
import rosbag
import os
import sys
import argparse

def check_delay(inbag):
  print '   Processing input bagfile: ', inbag
  delays = {}
  for topic, msg, t in rosbag.Bag(inbag,'r').read_messages():
    if msg._has_header:
      if topic not in delays:
        delays[topic] = []
      delay = abs((t - msg.header.stamp).to_sec())
      delays[topic].append(delay)
  max_len = max(len(topic) for topic in delays.keys())
  topics = delays.keys()
  topics.sort()
  for topic in topics:
    delay_list = delays[topic]
    dmin, dmax, dmean = min(delay_list), max(delay_list), sum(delay_list)/len(delay_list)
    print topic.ljust(max_len + 2), ": mean = ", dmean, ", min = ", dmin, ", max = ", dmax


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description='Checks the delay in a bagfile between publishing (recording) time and the time stamp in the header (if exists).')
  parser.add_argument('inbag', help='input bagfile')
  args = parser.parse_args()
  try:
    check_delay(args.inbag)
  except Exception, e:
    import traceback
    traceback.print_exc()
