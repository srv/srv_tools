#!/usr/bin/python

import roslib; roslib.load_manifest('srv_tools')
import rosbag
import rospy
import cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Reading bag filename from command line or roslaunch parameter.
import os
import sys

class ImageCreator():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        # Get parameters from arguments 
        bagfile = os.path.join(sys.path[0], sys.argv[1])
        stereo = sys.argv[2]
        image_topic = sys.argv[3]
        save_dir = os.path.join(sys.path[0], sys.argv[4])

        # Use a CvBridge to convert ROS images to OpenCV images so they can be saved.
        self.bridge = CvBridge()

        # Open bag file.
        with rosbag.Bag(bagfile, 'r') as bag:
            for topic, msg, t in bag.read_messages():
                if topic == stereo + "/left/" + image_topic:
                    try:
                        cv_image = self.bridge.imgmsg_to_cv(msg, "bgr8")
                    except CvBridgeError, e:
                        print e
                    timestr = str(msg.header.stamp.to_nsec())
                    image_name = str(save_dir)+"/left_"+timestr+".png"
                    cv.SaveImage(image_name, cv_image)
                    print "saved ", image_name
                if topic == stereo + "/right/" + image_topic:
                    try:
                        cv_image = self.bridge.imgmsg_to_cv(msg, "bgr8")
                    except CvBridgeError, e:
                        print e
                    timestr = str(msg.header.stamp.to_nsec())
                    image_name = str(save_dir)+"/right_"+timestr+".png"
                    cv.SaveImage(image_name, cv_image)
                    print "saved ", image_name

# Main function.    
if __name__ == '__main__':
  if len(sys.argv) < 5:
      print 'Usage:', sys.argv[0], '<bagfile> <stereo cam name> <img topic> <save dir>'
      sys.exit(0)
    # Go to class functions that do all the heavy lifting. Do error checking.
  try:
    image_creator = ImageCreator()
  except rospy.ROSInterruptException: pass
