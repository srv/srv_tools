#!/usr/bin/python
"""
Copyright (c) 2012,
Systems, Robotics and Vision Group
University of the Balearican Islands
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Systems, Robotics and Vision Group, University of
      the Balearican Islands nor the names of its contributors may be used to
      endorse or promote products derived from this software without specific
      prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""


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
    rospy.loginfo('   Processing input bagfile: %s', inbag)
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
    rospy.loginfo('%s : mean = %s, min = %s, max = %s, median = %s', topic.ljust(max_len + 2), dmean, dmin, dmax, dmedian)

if __name__ == "__main__":
  rospy.init_node('check_delay')
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
