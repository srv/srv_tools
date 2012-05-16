#!/usr/bin/python

PKG = 'bag_tools' # this package name

import roslib; roslib.load_manifest(PKG)
import rospy
import rosbag
import os
import sys
import argparse
import yaml
import sensor_msgs.msg

def change_camera_info(inbag,outbag,replacements):
  print '   Processing input bagfile: ', inbag
  print '  Writing to output bagfile: ', outbag
  # parse the replacements
  maps = {}
  for k, v in replacements.items():
    print 'Changing topic', k, 'to contain following info (header will not be changed):\n', v

  outbag = rosbag.Bag(outbag,'w')
  for topic, msg, t in rosbag.Bag(inbag,'r').read_messages():
    if topic in replacements:
      new_msg = replacements[topic]
      new_msg.header = msg.header
      msg = new_msg
    outbag.write(topic, msg, t)
  print 'Closing output bagfile and exit...'
  outbag.close();

def replacement(replace_string):
  pair = replace_string.split('=', 1)
  if len(pair) != 2:
    raise argparse.ArgumentTypeError("Replace string must have the form /topic=calib_file.yaml")
  if pair[0][0] != '/':
    pair[0] = '/'+pair[0]
  stream = file(pair[1], 'r')
  calib_data = yaml.load(stream)
  cam_info = sensor_msgs.msg.CameraInfo()
  cam_info.width = calib_data['image_width']
  cam_info.height = calib_data['image_height']
  cam_info.K = calib_data['camera_matrix']['data']
  cam_info.D = calib_data['distortion_coefficients']['data']
  cam_info.R = calib_data['rectification_matrix']['data']
  cam_info.P = calib_data['projection_matrix']['data']
  cam_info.distortion_model = calib_data['distortion_model']
  return pair[0], cam_info

if __name__ == "__main__":
  parser = argparse.ArgumentParser(description='Change camera info messages in a bagfile.')
  parser.add_argument('inbag', help='input bagfile')
  parser.add_argument('outbag', help='output bagfile')
  parser.add_argument('replacement', type=replacement, nargs='+', help='replacement in form "TOPIC=CAMERA_INFO_FILE", e.g. /stereo/left/camera_info=my_new_info.yaml')
  args = parser.parse_args()
  replacements = {}
  for topic, calib_file in args.replacement:
    replacements[topic] = calib_file
  try:
    change_camera_info(args.inbag, args.outbag, replacements)
  except Exception, e:
    import traceback
    traceback.print_exc()
