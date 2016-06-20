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
import tempfile
import subprocess
import glob
import shutil

def create_video(tmp_dir, args):
  rospy.loginfo('Using {} as working directory.'.format(tmp_dir))
  rospy.loginfo('Extracting images...')

  cmd = ["rosrun", "bag_tools", "extract_images" , tmp_dir, "jpg", args.topic] + args.inbag
  rospy.loginfo('    {}'.format(' '.join(cmd)))
  subprocess.call(cmd)

  rospy.loginfo("Renaming...")
  images = glob.glob(tmp_dir + '/*.jpg')
  images.sort()
  i = 1
  for image in images:
    shutil.move(image, tmp_dir + '/img-' + str(i) + '.jpg')
    i = i + 1

  rospy.loginfo('Creating video...')
  cmd = ["ffmpeg", "-f", "image2", "-r", str(args.fps), "-i", tmp_dir + "/img-%d.jpg", args.output]
  rospy.loginfo('    {}'.format(' '.join(cmd)))
  subprocess.call(cmd)


if __name__ == "__main__":
  rospy.init_node('make_video')
  import argparse
  parser = argparse.ArgumentParser(
      description=
        'Creates a video from sensor_msgs/Image messages from a bagfile. '
        'This script uses the extract_images binary to extract color images '
        'from bagfiles and calls ffmpeg afterwards to combine them together '
        'to form a video. Note that ffmpeg must be installed on your system.')
  parser.add_argument('topic', help='topic of the images to use')
  parser.add_argument('--output', help='name of the output video. Note that the file ending defines the codec to use.', default='video.mp4')
  parser.add_argument('--fps', help='frames per second in the output video, as long as codec supports this', type=int, default=20)
  parser.add_argument('inbag', help='input bagfile(s)', nargs='+')
  args = parser.parse_args()
  tmp_dir = tempfile.mkdtemp()
  try:
    create_video(tmp_dir, args)
  except Exception, e:
    import traceback
    traceback.print_exc()
  rospy.loginfo('Cleaning up temp files...')
  shutil.rmtree(tmp_dir)
