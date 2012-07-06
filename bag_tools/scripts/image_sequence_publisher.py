#!/usr/bin/python

PKG = 'bag_tools' # this package name

import roslib; roslib.load_manifest(PKG)
import rospy
import rosbag
import os
import sys
import argparse
import sensor_msgs.msg
import cv_bridge
import camera_info_parser
import glob
import cv
import argparse

def collect_image_files(image_dir):
  jpg_images = glob.glob(image_dir + '/*.jpg')
  jpg_images.sort()
  return jpg_images

def playback_images(image_dir,camera_info_file,publish_rate):
  cam_info = camera_info_parser.parse_yaml(camera_info_file)
  image_files = collect_image_files(image_dir)
  rospy.loginfo('Found %i images.',len(image_files))
  bridge = cv_bridge.CvBridge()
  rate = rospy.Rate(publish_rate)
  image_publisher = rospy.Publisher('camera/image', sensor_msgs.msg.Image)
  cam_info_publisher = rospy.Publisher('camera/camera_info', sensor_msgs.msg.CameraInfo)
  rospy.loginfo('Starting playback.')
  for image_file in image_files:
    if rospy.is_shutdown():
      break
    now = rospy.Time.now()
    image = cv.LoadImage(image_file)
    image_msg = bridge.cv_to_imgmsg(image, encoding='rgb8')
    image_msg.header.stamp = now
    image_msg.header.frame_id = "/camera"
    image_publisher.publish(image_msg)
    cam_info.header.stamp = now
    cam_info.header.frame_id = "/camera"
    cam_info_publisher.publish(cam_info)
    rate.sleep()
  rospy.loginfo('No more images left. Stopping.')

if __name__ == "__main__":
  parser = argparse.ArgumentParser(
      description='Publishes a set of images as if it was a real connected camera')
  parser.add_argument('-dir', metavar='IMAGE_DIR', required=True, help='folder where the images are stored')
  parser.add_argument('-info', metavar='CAMERA_INFO', required=True, help='camera info file path')
  parser.add_argument('-hz', metavar='HZ', default=10, help='publish rate in Hz')
  args = parser.parse_args()
  try:
    playback_images(args.dir, args.info, args.hz)
  except Exception, e:
    import traceback
    traceback.print_exc()
