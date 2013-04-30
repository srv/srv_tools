#!/usr/bin/env python

import roslib; roslib.load_manifest('plot_tools')
import sys
import pylab
import math
import numpy as np
import string
import random
import time
import ntpath
from matplotlib import pyplot
from mpl_toolkits.mplot3d import Axes3D

# Global variables
len_data = 0
first_iter = True
colors = ['g','r','b']

class Error(Exception):
  """ Base class for exceptions in this module. """
  pass

def real_time_plot(files):
  """
  Function to plot the data saved into the files in real time
  """
  global len_data, first_iter, colors

  for i,F in enumerate(files):

    # Load data
    data = pylab.loadtxt(F, delimiter=',', skiprows=1, usecols=(5,6,7))

    # Check if new data
    if (len_data!= len(data[:,0])):

      # Plot
      label = ntpath.basename(F)
      label = label[0:-4]
      ax.plot(data[:,0], data[:,1], data[:,2], colors[i], label=label)

      pyplot.draw()

      # Update globals
      len_data = len(data[:,0])

  if (first_iter == True):
    ax.legend()
    first_iter = False

if __name__ == "__main__":
  import argparse
  parser = argparse.ArgumentParser(
          description='Plot 3D graphics of odometry data files in real time.',
          formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument('ground_truth_files', 
          help='file with ground truth odometry', 
          nargs='+')
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
  timer.add_callback(real_time_plot, args.ground_truth_files)
  timer.start()
  
  pylab.show() 
