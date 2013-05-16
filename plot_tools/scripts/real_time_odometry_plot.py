#!/usr/bin/env python

import sys
import pylab
import math
import weakref
import os
import ntpath
from matplotlib import pyplot
from mpl_toolkits.mplot3d import Axes3D

# Global variables
first_iter = True
colors = ['g','r','b']
ax_plot = []

class Error(Exception):
  """ Base class for exceptions in this module. """
  pass

def real_time_plot(files):
  """
  Function to plot the data saved into the files in real time
  """
  global ax_plot, first_iter, colors

  # Remove all previous data
  for i in range(len(ax_plot)):
    l = ax_plot[i].pop(0)
    wl = weakref.ref(l)
    l.remove()
    del l  
  ax_plot = []

  # Re-plot
  for i,F in enumerate(files):

    # File sanity check
    if (F != "" and os.path.exists(F)):
      try:
        # Load data
        data = pylab.loadtxt(F, delimiter=',', skiprows=1, usecols=(5,6,7))
        label = ntpath.basename(F)
        label = label[0:-4]
        ax_temp = ax.plot(data[:,0], data[:,1], data[:,2], colors[i], label=label)
        ax_plot.append(ax_temp)
      except:
        print "Could not open or no data in ", F

  # Update graphics
  pyplot.draw()

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
          default='3000')
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
