#!/usr/bin/python
"""
Copyright (c) 2015,
Enrique Fernandez Perdomo
Clearpath Robotics, Inc.
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

from __future__ import print_function

import rosbag

import argparse
import os
import sys

def merge(inbags, outbag='output.bag', topics=None, exclude_topics=[], raw=True):
  # Open output bag file:
  try:
    out = rosbag.Bag(outbag, 'a' if os.path.exists(outbag) else 'w')
  except IOError as e:
    print('Failed to open output bag file %s!: %s' % (outbag, e.message), file=sys.stderr)
    return 127

  # Write the messages from the input bag files into the output one:
  for inbag in inbags:
    try:
      print('   Processing input bagfile: %s' % inbag)
      for topic, msg, t in rosbag.Bag(inbag, 'r').read_messages(topics=topics, raw=raw):
        if topic not in args.exclude_topics:
          out.write(topic, msg, t, raw=raw)
    except IOError as e:
      print('Failed to open input bag file %s!: %s' % (inbag, e.message), file=sys.stderr)
      return 127

  out.close()

  return 0


if __name__ == "__main__":
  parser = argparse.ArgumentParser(
      description='Merge multiple bag files into a single one.')
  parser.add_argument('inbag', help='input bagfile(s)', nargs='+')
  parser.add_argument('--output', help='output bag file', default='output.bag')
  parser.add_argument('--topics', help='topics to merge from the input bag files', nargs='+', default=None)
  parser.add_argument('--exclude_topics', help='topics not to merge from the input bag files', nargs='+', default=[])
  args = parser.parse_args()

  try:
    sys.exit(merge(args.inbag, args.output, args.topics, args.exclude_topics))
  except Exception, e:
    import traceback
    traceback.print_exc()
