#!/usr/bin/python
"""
Copyright (c) 2015,
Clearpath Robotics, Inc.
Enrique Fernandez Perdomo
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
import tf.transformations as tft
from geometry_msgs.msg import Transform, Vector3, Quaternion

import numpy
import argparse


def transform_vector3_msg_to_tf(msg):
  return tft.translation_matrix([msg.x, msg.y, msg.z])


def transform_tf_to_vector3_msg(tf):
    return Vector3(*tf)


def transform_quaternion_msg_to_tf(msg):
  return tft.quaternion_matrix([msg.x, msg.y, msg.z, msg.w])


def transform_tf_to_quaternion_msg(tf):
    q = tft.quaternion_from_euler(*rotation)
    return Quaternion(*q)


def transform_msg_to_tf(msg):
  translation = transform_vector3_msg_to_tf(msg.translation)
  rotation    = transform_quaternion_msg_to_tf(msg.rotation)

  return numpy.dot(rotation, translation)


def transform_tf_to_msg(tf):
    _, _, rotation, translation, _ = tft.decompose_matrix(tf)

    translation = transform_tf_to_vector3_msg(translation)
    rotation    = transform_tf_to_quaternion_msg(rotation)

    return Transform(translation, rotation)


def transform_tf(inbag, outbag, transform, frame_id, child_frame_id):
  rospy.loginfo('   Processing input bagfile: %s', inbag)
  rospy.loginfo('  Writing to output bagfile: %s', outbag)
  rospy.loginfo('            Transforming TF: %s -> %s' % (frame_id, child_frame_id))

  outbag = rosbag.Bag(outbag,'w')
  for topic, msg, t in rosbag.Bag(inbag, 'r').read_messages():
      if topic == "/tf":
          new_transforms = []
          for tf in msg.transforms:
              if tf.header.frame_id == frame_id and tf.child_frame_id == child_frame_id:
                  tf_transform = transform_msg_to_tf(tf.transform)
                  tf_transform = numpy.dot(tf_transform, transform)

                  tf.transform = transform_tf_to_msg(tf_transform)
              new_transforms.append(tf)
          msg.transforms = new_transforms
      outbag.write(topic, msg, t)

  rospy.loginfo('Closing output bagfile and exit...')
  outbag.close();


if __name__ == "__main__":
  parser = argparse.ArgumentParser(
      description='transform all transforms from the /tf topic that contain one of the given frame_ids in the header as parent.')

  parser.add_argument('-i', metavar='INPUT_BAGFILE', required=True, help='input bagfile')
  parser.add_argument('-o', metavar='OUTPUT_BAGFILE', required=True, help='output bagfile')
  parser.add_argument('-x', metavar='TRANSFORM_X', required=False, type=float, default=0.0, help='transform translation on x-axis [m]')
  parser.add_argument('-y', metavar='TRANSFORM_Y', required=False, type=float, default=0.0, help='transform translation on y-axis [m]')
  parser.add_argument('-z', metavar='TRANSFORM_Z', required=False, type=float, default=0.0, help='transform translation on z-axis [m]')
  parser.add_argument('-R', metavar='TRANSFORM_ROLL', required=False, type=float, default=0.0, help='transform rotation roll [deg]')
  parser.add_argument('-P', metavar='TRANSFORM_PITCH', required=False, type=float, default=0.0, help='transform rotation pitch [deg]')
  parser.add_argument('-Y', metavar='TRANSFORM_YAW', required=False, type=float, default=0.0, help='transform rotation yaw [deg]')
  parser.add_argument('-f', metavar='FRAME_ID', required=True, help='frame_id of the transform to change/transform from the /tf topic')
  parser.add_argument('-c', metavar='CHILD_FRAME_ID', required=True, help='child_frame_id of the transform to change/transform from the /tf topic')

  args = parser.parse_args()

  # Create transform:
  translation = numpy.array([args.x, args.y, args.z])
  rotation    = numpy.deg2rad([args.R, args.P, args.Y])

  transform = tft.compose_matrix(angles=rotation, translate=translation)

  try:
    transform_tf(args.i, args.o, transform, args.f, args.c)
  except Exception, e:
    import traceback
    traceback.print_exc()
