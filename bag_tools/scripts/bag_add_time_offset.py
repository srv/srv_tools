#!/usr/bin/python

PKG = 'bagfile_utils' # this package name

import roslib; roslib.load_manifest(PKG)
import rospy
import rosbag
import sensor_msgs.msg
import os
import sys

def fix_bagfile(inbag, outbag, left_info_url, right_info_url):
    print 'Processing input bagfile : ', inbag
    print 'Writing to output bagfile : ', outbag

    outbag = rosbag.Bag(outbag, 'w')

    time_offset = rospy.Duration(1e-6)

    for topic, msg, t in rosbag.Bag(inbag).read_messages():
        if topic == "MY_SUPER_TOPIC_WITH_WRONG_TIMESTAMPED_MESSAGES":
            outbag.write(topic, msg, t + time_offset)
        else:
            outbag.write(topic, msg, t)
    print 'Closing output bagfile and exit...'
    outbag.close()

if __name__ == "__main__":
    parser = OptionParser(usage="%prog INBAG OUTBAG",
                          description='Create a new bagfile from an existing one shifting time by hardcoded offset.')
    (options, args) = parser.parse_args()
    if len(args) != 2:
        parser.error('Wrong number of arguments')
    inbag = args[0]
    outbag = args[1]
    try:
        fix_bagfile(inbag, outbag)
    except Exception, e:
        import traceback
        traceback.print_exc()
