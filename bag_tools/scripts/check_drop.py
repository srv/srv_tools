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

import rospy
import rosbag

import numpy
import argparse

# Workaround to avoid issues with X11 rendering when running on background:
import matplotlib as mpl
mpl.use('Agg')

import matplotlib.pyplot as plt

def check_drop(inbags, plot_format='png'):
  # Retrieve msg time, bag time and sequence number for all topics and messages:
  msg_time = {}
  bag_time = {}
  seq = {}

  for inbag in inbags:
    rospy.loginfo('   Processing input bagfile: %s', inbag)
    for topic, msg, t in rosbag.Bag(inbag,'r').read_messages():
      if topic == "/tf":
        for transform in msg.transforms:
          key = "/tf: " + transform.header.frame_id + " -> " + transform.child_frame_id
          if key not in msg_time:
            msg_time[key] = []
            bag_time[key] = []
            seq[key] = []
          else:
            msg_time[key].append(transform.header.stamp.to_sec())
            bag_time[key].append(t.to_sec())
            seq[key].append(transform.header.seq)
      elif msg._has_header:
        key = topic
        if key not in msg_time:
          msg_time[key] = []
          bag_time[key] = []
          seq[key] = []
        else:
          msg_time[key].append(msg.header.stamp.to_sec())
          bag_time[key].append(t.to_sec())
          seq[key].append(msg.header.seq)

  # Convert lists to numpy.arrays:
  for key in msg_time.iterkeys():
    msg_time[key] = numpy.array(msg_time[key])
    bag_time[key] = numpy.array(bag_time[key])
    seq[key] = numpy.array(seq[key])

  # Compute differences:
  msg_time_diff = {}
  bag_time_diff = {}
  seq_diff = {}

  min_time = []
  for key in msg_time.iterkeys():
    msg_time_diff[key] = numpy.diff(msg_time[key])
    min_time.append(msg_time[key][0])
    bag_time_diff[key] = numpy.diff(bag_time[key])
    seq_diff[key] = numpy.diff(seq[key])

  min_time = numpy.min(min_time)

  # Compute min, max and mean differences:
  basename = inbags[0].replace('.bag', '')

  max_len = max(len(topic) for topic in msg_time_diff.keys())
  topics = msg_time_diff.keys()
  topics.sort()


  for topic in topics:
    msg_time_diff_topic = msg_time_diff[topic]
    bag_time_diff_topic = bag_time_diff[topic]
    seq_diff_topic = seq_diff[topic]

    if len(msg_time_diff_topic) == 0:
      rospy.logwarn('%s has no messages', topic.ljust(max_len + 2))
      continue

    msg_time_diff_min = numpy.min(msg_time_diff_topic)
    msg_time_diff_max = numpy.max(msg_time_diff_topic)
    msg_time_diff_mean = numpy.mean(msg_time_diff_topic)
    msg_time_diff_median = numpy.median(msg_time_diff_topic)

    bag_time_diff_min = numpy.min(bag_time_diff_topic)
    bag_time_diff_max = numpy.max(bag_time_diff_topic)
    bag_time_diff_mean = numpy.mean(bag_time_diff_topic)
    bag_time_diff_median = numpy.median(bag_time_diff_topic)

    seq_diff_min = numpy.min(seq_diff_topic)
    seq_diff_max = numpy.max(seq_diff_topic)
    seq_diff_mean = numpy.mean(seq_diff_topic)
    seq_diff_median = numpy.median(seq_diff_topic)

    rospy.loginfo('%s message time: mean = %s, min = %s, max = %s, median = %s', topic.ljust(max_len + 2), msg_time_diff_mean, msg_time_diff_min, msg_time_diff_max, msg_time_diff_median)
    rospy.loginfo('%s bag     time: mean = %s, min = %s, max = %s, median = %s', topic.ljust(max_len + 2), bag_time_diff_mean, bag_time_diff_min, bag_time_diff_max, bag_time_diff_median)
    rospy.loginfo('%s          seq: mean = %s, min = %s, max = %s, median = %s', topic.ljust(max_len + 2), seq_diff_mean, seq_diff_min, seq_diff_max, seq_diff_median)

    # Create and save plots:
    mt = msg_time[topic][1:] - min_time  

    try:
      fig = plt.figure()
      fig.set_size_inches(20, 15)

      plt.subplot(311)
      plt.title(topic + ' - Message time difference [s]')
      plt.plot(mt, msg_time_diff_topic,'b')
      plt.plot([mt[0], mt[-1]], [msg_time_diff_min, msg_time_diff_min], 'r--')
      plt.plot([mt[0], mt[-1]], [msg_time_diff_max, msg_time_diff_max], 'r--')
      plt.plot([mt[0], mt[-1]], [msg_time_diff_mean, msg_time_diff_mean], 'g')

      plt.subplot(312)
      plt.title(topic + ' - Bag time difference [s]')
      plt.plot(mt, bag_time_diff_topic,'b')
      plt.plot([mt[0], mt[-1]], [bag_time_diff_min, bag_time_diff_min], 'r--')
      plt.plot([mt[0], mt[-1]], [bag_time_diff_max, bag_time_diff_max], 'r--')
      plt.plot([mt[0], mt[-1]], [bag_time_diff_mean, bag_time_diff_mean], 'g')

      plt.subplot(313)
      plt.title(topic + ' - Sequence number difference')
      plt.plot(mt, seq_diff_topic, 'b')
      plt.plot([mt[0], mt[-1]], [seq_diff_min, seq_diff_min], 'r--')
      plt.plot([mt[0], mt[-1]], [seq_diff_max, seq_diff_max], 'r--')
      plt.plot([mt[0], mt[-1]], [seq_diff_mean, seq_diff_mean], 'g')


      plt.savefig(basename + topic.replace('/', '_').replace(' ', '_').replace(':', '_') + '.' + plot_format)
      plt.close(fig)
    except OverflowError as e:
      rospy.logerr('%s: Failed to save plots as %s image files (try other format, e.g. svg): %s', topic.ljust(max_len + 2), plot_format, e.message)

if __name__ == "__main__":
  rospy.init_node('check_drop', anonymous=True)

  parser = argparse.ArgumentParser(
      description='Checks the header.seq and header.stamp difference to detect '
                  'message dropping in a bagfile. Prints out min, max and mean '
                  'differences and all values are shown on a plot.')
  parser.add_argument('inbag', help='input bagfile(s)', nargs='+')
  parser.add_argument('--plot_format', help='output plot format', default='png')
  args = parser.parse_args()
  try:
    check_drop(args.inbag, args.plot_format)
  except Exception, e:
    import traceback
    traceback.print_exc()
