#!/usr/bin/python

PKG = 'bag_tools' # this package name

import roslib; roslib.load_manifest(PKG)
import rospy
import rosbag
import os
import sys
import argparse

def check_delay(inbags):
  delays = {}
  for inbag in inbags:
    print '   Processing input bagfile: ', inbag
    for topic, msg, t in rosbag.Bag(inbag,'r').read_messages():
      if topic == "/tf":
        for transform in msg.transforms:
          delay = abs((t - transform.header.stamp).to_sec())
          key = "/tf: " + transform.header.frame_id + " -> " + transform.child_frame_id
          if key not in delays:
            delays[key] = []
          delays[key].append(delay)
      elif msg._has_header:
        key = topic
        if key not in delays:
          delays[key] = []
        delay = abs((t - msg.header.stamp).to_sec())
        delays[key].append(delay)
  max_len = max(len(topic) for topic in delays.keys())
  topics = delays.keys()
  topics.sort()
  for topic in topics:
    delay_list = delays[topic]
    delay_list.sort()
    dmin, dmax, dmean = min(delay_list), max(delay_list), sum(delay_list)/len(delay_list)
    dmedian = delay_list[len(delay_list)/2]
    print topic.ljust(max_len + 2), ": mean = ", dmean, ", min = ", dmin, ", max = ", dmax, ", median = ", dmedian


if __name__ == "__main__":
  parser = argparse.ArgumentParser(
      description='Checks the delay in a bagfile between publishing (recording) '
                  'time and the time stamp in the header (if exists). Prints '
                  'out min, max and mean delays.')
  parser.add_argument('inbag', help='input bagfile(s)', nargs='+')
  args = parser.parse_args()
  try:
    check_delay(args.inbag)
  except Exception, e:
    import traceback
    traceback.print_exc()
