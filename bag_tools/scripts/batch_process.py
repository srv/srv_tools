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
import os
import sys
import argparse
import glob
import subprocess

def process(in_dir,out_dir,command):
    bagfiles = glob.glob(in_dir + "/*.bag")
    for bagfile in bagfiles:
        outbag = out_dir + "/" + os.path.basename(bagfile)
        if os.path.exists(outbag):
            rospy.loginfo('%s exists, skipping.', outbag)
        else:
            cmd = command.split()
            cmd.append("-i")
            cmd.append(bagfile)
            cmd.append("-o")
            cmd.append(outbag)
            subprocess.check_call(cmd)

if __name__ == "__main__":
  rospy.init_node('batch_process')
  parser = argparse.ArgumentParser(
      description='batch processes all bagfiles in INPUT_DIR, writing output to OUTPUT_DIR by calling given command with -i and -o arguments.')
  parser.add_argument('-i', metavar='INPUT_DIR', required=True, help='input directory with input bagfiles')
  parser.add_argument('-o', metavar='OUTPUT_DIR', required=True, help='output directory for bagfiles')
  parser.add_argument('-c', metavar='COMMAND', required=True, help='command to execute with each bagfile as input and with same name in output OUTPUT_DIR')
  args = parser.parse_args()

  try:
    process(args.i,args.o,args.c)
  except Exception, e:
    import traceback
    traceback.print_exc()
