#!/usr/bin/python3

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


import os
import rospy
import rosbag
import argparse

from typing import List


def sortBags(in_bag_paths: List[str],
             out_bag_paths: List[str]) -> None:
    r"""
    Sort the topics of some bagfiles by their timestamps.

    Parameters
    ----------
    in_bag_paths : List[str]
        Input bagfile path(s)
    out_bag_paths : List[str]
        Output bagfile path(s)

    Returns
    -------
    out : None

    Raises
    ------
    FileNotFoundError
        If the input bagfile does not exist.
    NotADirectoryError
        If any of the output directories do not exist.
    MemoryError
        If the bagfile is too large for the RAM.
    """
    for in_bag_path, out_bag_path in zip(in_bag_paths, out_bag_paths):

        # Sanity check.
        print(f"Reading and processing bagfile: {in_bag_path}")
        if not os.path.isfile(in_bag_path):
            raise FileNotFoundError(f"The input bagfile: {in_bag_path} does not exist!")
        
        print(f"Writing bagfile: {out_bag_path}")
        outdir = os.path.dirname(out_bag_path)
        if outdir and not os.path.isdir(outdir):
            raise NotADirectoryError(f"The output directory: {outdir} does not exist!")
        
        with rosbag.Bag(in_bag_path, "r") as inbag:
            print("Sorting messages in memory... (this may take a while)")

            try:
                all_msgs = sorted(inbag.read_messages(), key = lambda x: x[1].header.stamp if hasattr(x[1], "header") and x[1].header.stamp.to_sec() > 0 else x[2])
            except MemoryError:
                raise MemoryError("The bagfile is too large for the RAM. You need a fragment-by-fragment approach.")
        
        with rosbag.Bag(out_bag_path, "w") as outbag:
            for topic, msg, t in all_msgs:
                if hasattr(msg, "header") and msg.header.stamp.to_sec() > 0:
                    outbag.write(topic, msg, msg.header.stamp)
                else:
                    outbag.write(topic, msg, t) 

        print(f"Bagfile: {out_bag_path} written successfully!")

        
def main():
    r"""
    Main entry point.
    """
    # Init ros node.
    rospy.init_node("sort_bags", anonymous = True)

    # Command-line interface.
    parser = argparse.ArgumentParser(description = "Sort the topics of some bagfiles by their timestamps.",
                                     formatter_class = argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("-ibps", "--in_bag_paths", help = "Input bagfile path(s)", nargs = "+", required = True)
    parser.add_argument("-obps", "--out_bag_paths", help = "Output bagfile path(s)", nargs = "+", required = True)
    args = parser.parse_args()

    # Sanity check.
    if len(args.in_bag_paths) != len(args.out_bag_paths):
        raise ValueError(f"The number of input bagfiles ({len(args.in_bag_paths)}) is different of the number of output bagfiles ({len(args.out_bag_paths)})")

    try:
        sortBags(args.in_bag_paths, args.out_bag_paths)
    except Exception as e:
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()