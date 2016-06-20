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

def change_topics(inbag,outbag,replacements):
  rospy.loginfo('   Processing input bagfile: %s', inbag)
  rospy.loginfo('  Writing to output bagfile: %s', outbag)
  rospy.loginfo('               Replacements: %s', replacements)

  outbag = rosbag.Bag(outbag,'w')
  for topic, msg, t in rosbag.Bag(inbag,'r').read_messages():
    if topic in replacements:
        topic = replacements[topic]
    outbag.write(topic, msg, t)
  rospy.loginfo('Closing output bagfile and exit...')
  outbag.close();

def replacement(replace_string):
  pair = replace_string.split('=', 1)
  if len(pair) != 2:
    raise argparse.ArgumentTypeError("Replace string must have the form /topic=/new_topic")
  if pair[0][0] != '/':
    pair[0] = '/'+pair[0]
  if pair[1][0] != '/':
    pair[1] = '/'+pair[1]
  return pair[0], pair[1]

if __name__ == "__main__":
  rospy.init_node('change_topics')
  parser = argparse.ArgumentParser(
      description='changes topic names inside a bagfile.')
  parser.add_argument('inbag', help='input bagfile')
  parser.add_argument('outbag', help='output bagfile')
  parser.add_argument('replacement', type=replacement, nargs='+', help='replacement in form "TOPIC=NEW_TOPIC", e.g. /laser1/scan=/laser2/scan')
  args = parser.parse_args()
  replacements = {}
  for topic, new_topic in args.replacement:
    replacements[topic] = new_topic

  try:
    change_topics(args.inbag,args.outbag,replacements)
  except Exception, e:
    import traceback
    traceback.print_exc()

