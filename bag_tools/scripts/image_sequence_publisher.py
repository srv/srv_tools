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
import sensor_msgs.msg
import cv_bridge
import camera_info_parser
import glob
import cv2
import numpy as np
from numpy import genfromtxt
import math
import tf
import os

fake_green = False

def collect_image_files(image_dir,file_pattern):
  images = glob.glob(image_dir + '/' + file_pattern)
  images.sort()
  #images = images[:1101]
  #images = images[1102:2202]
  # images = images[2203:]
  return images

def collect_poses(file):
  poses = genfromtxt(file, delimiter=',')
  return poses

def playback_images(image_dir,file_pattern,camera_info_file,pose_file,publish_rate):
  if camera_info_file != "":
    cam_info = camera_info_parser.parse_yaml(camera_info_file)
    publish_cam_info = True
  else:
    publish_cam_info = False
  if pose_file != "":
    poses = collect_poses(pose_file)
    publish_poses = True
  else:
    publish_poses = False
  image_files = collect_image_files(image_dir,file_pattern)
  rospy.loginfo('Found %i images.',len(image_files))
  bridge = cv_bridge.CvBridge()
  rate = rospy.Rate(publish_rate)
  image_publisher = rospy.Publisher('camera/image_color', sensor_msgs.msg.Image, queue_size = 5)
  if publish_cam_info:
    cam_info_publisher = rospy.Publisher('camera/camera_info', sensor_msgs.msg.CameraInfo, queue_size = 5)
  if publish_poses:
    tf_pose_publisher = tf.TransformBroadcaster()
  rospy.loginfo('Starting playback.')
  for image_file in image_files:
    if rospy.is_shutdown():
      break
    now = rospy.Time.now()
    image = cv2.imread(image_file)
    if fake_green:
      image[:,:,0] = 0
      image[:,:,2] = 0
    image_msg = bridge.cv2_to_imgmsg(np.asarray(image[:,:]), encoding='bgr8')
    image_msg.header.stamp = now
    image_msg.header.frame_id = "/camera"
    image_publisher.publish(image_msg)
    if publish_cam_info:
      cam_info.header.stamp = now
      cam_info.header.frame_id = "/camera"
      cam_info_publisher.publish(cam_info)
    if publish_poses:
      img_name = os.path.basename(image_file)
      idx = int(''.join(x for x in img_name if x.isdigit())) + 1
      tf_pose_publisher.sendTransform((poses[idx, 1], poses[idx, 2], poses[idx, 3]),
                     tf.transformations.quaternion_from_euler(math.radians(poses[idx, 4]), math.radians(poses[idx, 5]), math.radians(poses[idx, 6])),
                     now,
                     'dvl',
                     'world')
      # print img_name + ' =? ' + str(poses[idx, 0]) + " pose (" + str(poses[idx, 1]) + ", " + str(poses[idx, 2]) + ", " + str(poses[idx, 3]) + ") orientation (" + str(poses[idx, 4]) + ", " + str(poses[idx, 5]) + ", " + str(poses[idx, 6]) + ")"
    rate.sleep()
  rospy.loginfo('No more images left. Stopping.')

if __name__ == "__main__":
  rospy.init_node('image_sequence_publisher')
  try:
    image_dir = rospy.get_param("~image_dir")
    file_pattern = rospy.get_param("~file_pattern")
    camera_info_file = rospy.get_param("~camera_info_file", "")
    pose_file = rospy.get_param("~pose_file", "")
    frequency = rospy.get_param("~frequency", 10)
    fake_green = rospy.get_param("~fake_green", False)
    playback_images(image_dir, file_pattern, camera_info_file, pose_file, frequency)
  except KeyError as e:
    rospy.logerr('Required parameter missing: %s', e)
  except Exception, e:
    import traceback
    traceback.print_exc()
