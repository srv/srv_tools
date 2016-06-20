#!/usr/bin/python
"""
Created from image_sequence_publisher.py
Mathieu Labbe

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
import sensor_msgs.msg
import cv_bridge
import camera_info_parser
import glob
import cv
import numpy as np
import re

def natural_sort(l):
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [ convert(c) for c in re.split('([0-9]+)', key) ]
    return sorted(l, key = alphanum_key)

def collect_image_files(image_dir,file_pattern):
  images = glob.glob(image_dir + '/' + str(file_pattern))
  images = natural_sort(images)
  return images

def playback_images(image_dir_l, image_dir_r, file_pattern, camera_info_file_l, camera_info_file_r, publish_rate):
  frame_id = "/camera"
  if camera_info_file_l != "":
    cam_info_l = camera_info_parser.parse_yaml(camera_info_file_l)
    frame_id = cam_info_l.header.frame_id
    publish_cam_info_l = True
  else:
    publish_cam_info_l = False
  if camera_info_file_r != "":
    cam_info_r = camera_info_parser.parse_yaml(camera_info_file_r)
    publish_cam_info_r = True
  else:
    publish_cam_info_r = False

  image_files_l = collect_image_files(image_dir_l, file_pattern)
  image_files_r = collect_image_files(image_dir_r, file_pattern)
  rospy.loginfo('Found %i left images.',len(image_files_l))
  rospy.loginfo('Found %i right images.',len(image_files_r))

  bridge = cv_bridge.CvBridge()
  rate = rospy.Rate(publish_rate)
  image_publisher_l = rospy.Publisher('stereo_camera/left/image_raw', sensor_msgs.msg.Image, queue_size = 5)
  image_publisher_r = rospy.Publisher('stereo_camera/right/image_raw', sensor_msgs.msg.Image, queue_size = 5)
  if publish_cam_info_l:
      cam_info_publisher_l = rospy.Publisher('stereo_camera/left/camera_info', sensor_msgs.msg.CameraInfo, queue_size = 5)
  if publish_cam_info_r:
      cam_info_publisher_r = rospy.Publisher('stereo_camera/right/camera_info', sensor_msgs.msg.CameraInfo, queue_size = 5)

  rospy.loginfo('Starting playback.')
  for image_file_l, image_file_r in zip(image_files_l, image_files_r):
    if rospy.is_shutdown():
      break
    now = rospy.Time.now()
    image_l = cv.LoadImage(image_file_l)
    image_r = cv.LoadImage(image_file_r)
    image_msg = bridge.cv2_to_imgmsg(np.asarray(image_l[:,:]), encoding='bgr8')
    image_msg.header.stamp = now
    image_msg.header.frame_id = frame_id
    image_publisher_l.publish(image_msg)
    image_msg = bridge.cv2_to_imgmsg(np.asarray(image_r[:,:]), encoding='bgr8')
    image_msg.header.stamp = now
    image_msg.header.frame_id = frame_id
    image_publisher_r.publish(image_msg)
    if publish_cam_info_l:
      cam_info_l.header.stamp = now
      cam_info_publisher_l.publish(cam_info_l)
    if publish_cam_info_r:
      cam_info_r.header.stamp = now
      cam_info_r.header.frame_id = frame_id
      cam_info_publisher_r.publish(cam_info_r)
    rate.sleep()
  rospy.loginfo('No more images left. Stopping.')

if __name__ == "__main__":
  rospy.init_node('image_sequence_publisher')
  try:
    image_dir_l = rospy.get_param("~image_dir_left")
    image_dir_r = rospy.get_param("~image_dir_right")
    file_pattern = rospy.get_param("~file_pattern")
    camera_info_file_l = rospy.get_param("~camera_info_file_left", "")
    camera_info_file_r = rospy.get_param("~camera_info_file_right", "")
    frequency = rospy.get_param("~frequency", 10)
    playback_images(image_dir_l, image_dir_r, file_pattern, camera_info_file_l, camera_info_file_r, frequency)
  except KeyError as e:
    rospy.logerr('Required parameter missing: %s', e)
  except Exception, e:
    import traceback
    traceback.print_exc()
