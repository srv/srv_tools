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

def fix_bagfile(inbag, outbag, topics, offset):
    rospy.loginfo('      Processing input bagfile: %s', inbag)
    rospy.loginfo('     Writing to output bagfile: %s', outbag)
    rospy.loginfo('               Changing topics: %s', topics)
    rospy.loginfo('   Changing publishing time by: %s', offset)

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
    rospy.loginfo('Closing output bagfile.')
    outbag.close()
    rospy.loginfo('Changed the following:')
    for k, v in count.items():
      rospy.loginfo( '%s:%s messages.',k,v)

if __name__ == "__main__":
  rospy.init_node('bag_add_time_offset')
  parser = argparse.ArgumentParser(
      description='Shift the publishing time of given topics in input bagfile.')
  parser.add_argument('-o', metavar='OUTPUT_BAGFILE', required=True, help='output bagfile')
  parser.add_argument('-i', metavar='INPUT_BAGFILE', required=True, help='input bagfile(s)')
  parser.add_argument('-of', metavar='OFFSET', required=True, type=float, help='time offset to add in seconds')
  parser.add_argument('-t', metavar='TOPIC', required=True, help='topic(s) to change', nargs='+')
  args = parser.parse_args()
  try:
    for bagfile in args.i:
      fix_bagfile(bagfile, args.o, args.t, args.of)
  except Exception as e:
    import traceback
    traceback.print_exc()
