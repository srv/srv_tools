#!/usr/bin/env python

import roslib; roslib.load_manifest('plot_tools')
import sys
import pylab
import math
import numpy as np
import string
import random
import time
from matplotlib import pyplot
from mpl_toolkits.mplot3d import Axes3D

# Global variables
len_gt = 0
len_odom_1 = 0
len_odom_2 = 0
first_iter = True

class Error(Exception):
  """ Base class for exceptions in this module. """
  pass

def load_data(gt_file, odom_file_1, odom_file_2):
  """
  Function to load the data from files
  """
  gt      = pylab.loadtxt(gt_file, delimiter=',', skiprows=1, 
          usecols=(5,6,7))
  odom_1  = pylab.loadtxt(odom_file_1, delimiter=',', skiprows=1, 
          usecols=(5,6,7))
  odom_2  = pylab.loadtxt(odom_file_2, delimiter=',', skiprows=1, 
          usecols=(5,6,7))
  odom_2 = odom_2[15:-1,:]
  return gt, odom_1, odom_2

def real_time_plot(gt_file, odom_file_1, odom_file_2):
  """
  Function to plot the data saved into the files in real time
  """
  global len_gt, len_odom_1, len_odom_2, first_iter

  # Load data
  gt, odom_1, odom_2 = load_data(gt_file, odom_file_1, odom_file_2)

  # Check if new data
  if (len_gt != len(gt[:,0]) or len_odom_1 != len(odom_1[:,0]) or len_odom_2 != len(odom_2[:,0])):

    # Plot
    ax.plot(gt[:,0], gt[:,1], gt[:,2], 'g', label='Ground Truth')
    ax.plot(odom_1[:,0], odom_1[:,1], odom_1[:,2], 'r', label='Viso2')
    ax.plot(odom_2[:,0], odom_2[:,1], odom_2[:,2], 'b', label='Meskf')

    if (first_iter == True):
      ax.legend()
      first_iter = False

    pyplot.draw()

    # Update globals
    len_gt = len(gt[:,0])
    len_odom_1 = len(odom_1[:,0])
    len_odom_2 = len(odom_2[:,0])

if __name__ == "__main__":
  import argparse
  parser = argparse.ArgumentParser(
          description='Plot 3D graphics of odometry data files in real time.',
          formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument('ground_truth_file',
          help='file with ground truth odometry')
  parser.add_argument('odometry_file_1',
          help='file with odometry data 1',
          default='')
  parser.add_argument('odometry_file_2',
          help='file with odometry data 2',
          default='')
  parser.add_argument('-ts','--time-step',
          help='update frequency (in milliseconds)',
          default='200')
  args = parser.parse_args()

  # Init figure
  fig = pylab.figure(1)
  ax = Axes3D(fig)
  ax.grid(True)
  ax.set_title("Realtime Odometry Plot")
  ax.set_xlabel("X")
  ax.set_ylabel("Y")
  ax.set_zlabel("Z")

  timer = fig.canvas.new_timer(interval=args.time_step)
  timer.add_callback(real_time_plot, args.ground_truth_file, args.odometry_file_1, args.odometry_file_2)
  timer.start()
  
  pylab.show() 
