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
import cv

def collect_image_files(image_dir,file_pattern):
  images = glob.glob(image_dir + '/' + file_pattern)
  images.sort()
  return images

def playback_images(image_dir,file_pattern,camera_info_file,publish_rate):
  if camera_info_file != "":
    cam_info = camera_info_parser.parse_yaml(camera_info_file)
    publish_cam_info = True
  else:
    publish_cam_info = False
  image_files = collect_image_files(image_dir,file_pattern)
  rospy.loginfo('Found %i images.',len(image_files))
  bridge = cv_bridge.CvBridge()
  rate = rospy.Rate(publish_rate)
  image_publisher = rospy.Publisher('camera/image', sensor_msgs.msg.Image)
  if publish_cam_info:
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
    if publish_cam_info:
      cam_info.header.stamp = now
      cam_info.header.frame_id = "/camera"
      cam_info_publisher.publish(cam_info)
    rate.sleep()
  rospy.loginfo('No more images left. Stopping.')

if __name__ == "__main__":
  rospy.init_node('image_sequence_publisher')
  try:
    image_dir = rospy.get_param("~image_dir")
    file_pattern = rospy.get_param("~file_pattern")
    camera_info_file = rospy.get_param("~camera_info_file", "")
    frequency = rospy.get_param("~frequency", 10)
    playback_images(image_dir, file_pattern, camera_info_file, frequency)
  except KeyError as e:
    rospy.logerr('Required parameter missing: %s', e)
  except Exception, e:
    import traceback
    traceback.print_exc()
