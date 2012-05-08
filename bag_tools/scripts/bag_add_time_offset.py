#!/usr/bin/python

PKG = 'bag_tools' # this package name

import roslib; roslib.load_manifest(PKG)
import rospy
import rosbag
import os
import sys
from optparse import OptionParser

def fix_bagfile(inbag, outbag, topics, offset):
    print '   Processing input bagfile: ', inbag
    print '  Writing to output bagfile: ', outbag
    print '            Changing topics: ', topics
    print 'Changing publishing time by: ', offset

    outbag = rosbag.Bag(outbag, 'w')

    time_offset = rospy.Duration(offset)

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
    parser = OptionParser(usage="%prog INBAG OUTBAG OFFSET TOPICS",
                          description='Shift the publishing time of given topics in input bagfile.')
    (options, args) = parser.parse_args()
    if len(args) < 4:
        parser.error('Wrong number of arguments')
    inbag = args[0]
    outbag = args[1]
    offset = args[2]
    topics = []
    for i in len(args) - 3:
      topics.append(args[3+i])
    try:
        fix_bagfile(inbag, outbag, topics, offset)
    except Exception, e:
        import traceback
        traceback.print_exc()
