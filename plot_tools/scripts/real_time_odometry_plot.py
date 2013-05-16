#!/usr/bin/env python
import roslib; roslib.load_manifest('plot_tools')
import sys
import pylab
import math
import weakref
import numpy as np
import os
import ntpath
from matplotlib import pyplot
from mpl_toolkits.mplot3d import Axes3D
import tf
import tf.transformations as tf

# Global variables
first_iter = True
colors = ['g','r','b']
ax_plot = []
ax_roll = []
ax_pitch = []
ax_yaw = []

class Error(Exception):
  """ Base class for exceptions in this module. """
  pass

def real_time_plot(files):
  """
  Function to plot the data saved into the files in real time
  """
  global ax_plot, ax_roll, ax_pitch, ax_yaw, first_iter, colors

  # Remove all previous data
  clear_plots()

  # Re-plot
  for i,F in enumerate(files):

    # File sanity check
    if (F != "" and os.path.exists(F)):
      try:
        # Load data
        data = pylab.loadtxt(F, delimiter=',', skiprows=1, usecols=(5,6,7,8,9,10,11))
        label = ntpath.basename(F)
        label = label[0:-4]
        # Init data in zero
        initial = []
        initial.append(data[0,:])
        initial =  np.array(initial)
        initial[0,6] = initial[0,6] - 1
        data = data - initial[0,:]
        # Plot position
        ax_temp = ax.plot(data[:,0], data[:,1], data[:,2], colors[i], label=label)
        ax_plot.append(ax_temp)

        # Compute orientation
        roll = []
        pitch = []
        yaw = []
        for n in range(len(data)):
          pose = to_transform(data[n,:])
          r, p, y = tf.euler_from_matrix(pose)
          roll.append(r)
          pitch.append(p)
          yaw.append(y)
        roll = np.array(roll)
        pitch = np.array(pitch)
        yaw = np.array(yaw)
        # Plot orientation
        dist = trajectory_distances(data)
        ax_r_temp = ax_r.plot(dist, roll, colors[i], label=label)
        ax_p_temp = ax_p.plot(dist, pitch, colors[i], label=label)
        ax_y_temp = ax_y.plot(dist, yaw, colors[i], label=label)
        ax_roll.append(ax_r_temp)
        ax_pitch.append(ax_p_temp)
        ax_yaw.append(ax_y_temp)
      except:
        print "Could not open or no data in ", F

  # Update graphics
  fig1.canvas.draw()
  fig2.canvas.draw()

  if (first_iter == True):
    ax.legend()
    #ax_roll.legend()
    first_iter = False

def clear_plots():
  global ax_plot, ax_roll, ax_pitch, ax_yaw

  for i in range(len(ax_plot)):
    l = ax_plot[i].pop(0)
    wl = weakref.ref(l)
    l.remove()
    del l 
  for i in range(len(ax_roll)):
    l = ax_roll[i].pop(0)
    wl = weakref.ref(l)
    l.remove()
    del l  
  for i in range(len(ax_pitch)):
    l = ax_pitch[i].pop(0)
    wl = weakref.ref(l)
    l.remove()
    del l
  for i in range(len(ax_yaw)):
    l = ax_yaw[i].pop(0)
    wl = weakref.ref(l)
    l.remove()
    del l
  ax_plot = []
  ax_roll = []
  ax_pitch = []   
  ax_yaw = []

def to_transform(data_point):
    t = [data_point[0], data_point[1], data_point[2]]
    q = [data_point[3], data_point[4], data_point[5], data_point[6]]
    rot_mat = tf.quaternion_matrix(q)
    trans_mat = tf.translation_matrix(t)
    return tf.concatenate_matrices(trans_mat, rot_mat)

def trajectory_distances(data):
  """
  Function to compute the trajectory distances from a dataset where
  each row contains the transformation matrix.
  """
  dist = []
  dist.append(0)
  for i in range(len(data) - 1):
    dist.append(dist[i] + calc_dist(data[i, :], data[i + 1, : ]))
  return dist

def calc_dist_xyz(data_point1, data_point2):
  xdiff = data_point1[0] - data_point2[0]
  ydiff = data_point1[1] - data_point2[1]
  zdiff = data_point1[2] - data_point2[2]
  return xdiff, ydiff, zdiff

def calc_dist(data_point1, data_point2):
  xdiff, ydiff, zdiff = calc_dist_xyz(data_point1, data_point2)
  return math.sqrt(xdiff*xdiff + ydiff*ydiff + zdiff*zdiff)

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

  # Position figure
  fig1 = pylab.figure(1)
  ax = Axes3D(fig1)
  ax.grid(True)
  ax.set_title("Realtime Odometry Plot")
  ax.set_xlabel("X")
  ax.set_ylabel("Y")
  ax.set_zlabel("Z")

  # Orientation figure
  fig2 = pyplot.figure(2)
  ax_r = pyplot.subplot(311)
  ax_p = pyplot.subplot(312)
  ax_y = pyplot.subplot(313)
  ax_r.set_ylabel("Roll (rad)")
  ax_p.set_ylabel("Pitch (rad)")
  ax_y.set_ylabel("Yaw (rad)")
  ax_y.set_xlabel("Total trajectory distance (m)")

  # Update timer
  timer = fig1.canvas.new_timer(interval=args.time_step)
  timer.add_callback(real_time_plot, args.ground_truth_files)
  timer.start()
  
  pylab.show() 
