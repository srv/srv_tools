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

import os
import rospy
import argparse
import roslib; roslib.load_manifest(PKG)
from change_frame_id import change_frame_id

if __name__ == "__main__":

    '''
    CALL : python change_frame_id.py -i path/to/in/folder -o path/to/out/folder -f desired_frame_id -t topic_to_change1 topic_to_change2 topic_to_change3 ...

    '''

    rospy.init_node('change_frame_id')
    parser = argparse.ArgumentParser(
        description='Create a new bagfile from an existing one replacing the frame id of requested topics.')
    parser.add_argument('-i', metavar = 'INPUT_PATH', required = True, help = 'input folder')
    parser.add_argument('-o', metavar = 'OUTPUT_PATH', required = True, help = 'output folder')
    parser.add_argument('-f', metavar = 'FRAME_ID', required = True, help = 'desired frame_id name in the topics')
    parser.add_argument('-t', metavar = 'TOPIC', required = True, nargs='+', help='topic(s) to change')
    args = parser.parse_args()

    if not os.path.exists(args.o):
      os.makedirs(args.o)

    for item in sorted(os.listdir(args.i)):

      file_path_in = os.path.join(args.i, item)
      file_path_out = os.path.join(args.o, item)

      if os.path.isfile(file_path_in):
          if ".bag" in item:
              print("working on file: " + str(item))

              try:
                  change_frame_id(file_path_in, file_path_out, args.f, args.t)
              except Exception:
                  import traceback
                  traceback.print_exc()