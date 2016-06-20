#!/usr/bin/env python
import roslib; roslib.load_manifest('stereo_slam')
import pylab
import numpy as np
from matplotlib import pyplot
from mpl_toolkits.mplot3d import Axes3D

class Error(Exception):
  """ Base class for exceptions in this module. """
  pass

def get_xyz(gps_point):
  lat = gps_point[3]*np.pi/180 #converting to radians.
  lon = gps_point[4]*np.pi/180 #converting to radians.
  alt = gps_point[5]*np.pi/180 #converting to radians.
  a = 6378137.0 # earth semimajor axis in meters.
  f = 1/298.257223563 # reciprocal flattening.
  e2 = 2*f - np.power(f,2) # eccentricity squared.

  chi = np.sqrt(1-e2 * np.power(np.sin(lat),2));
  x = (a/chi +alt) * np.cos(lat) * np.cos(lon);
  y = (a/chi +alt) * np.cos(lat) * np.sin(lon);
  z = (a*(1-e2)/chi + alt) * np.sin(lat);
  return x, y, z

if __name__ == "__main__":
  rospy.init_node('gps_to_std_gt')
  import argparse
  parser = argparse.ArgumentParser(
          description='Convert gps/fix topic to standard ground truth file',
          formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument('gps_topic_file',
          help='file with the gps/fix topic. Saved with "rostopic echo -p /gps/fix > data/anselm_turmeda/gt_gps.txt"')
  parser.add_argument('output_file',
          help='output file where the standard ground truth values will be saved.')
  args = parser.parse_args()

  # Read the
  gps_topic = pylab.loadtxt(args.gps_topic_file, delimiter=',', skiprows=1, usecols=(0,1,2,6,7,8))

  # Write the x, y, z data to the output file
  with open(args.output_file, 'w') as outfile:
    outfile.write("%time,field.header.seq,field.header.stamp,field.header.frame_id,field.child_frame_id,x,y,z,qx,qy,qz,qw\n")
    x0, y0, z0 = get_xyz(gps_topic[0])
    gt = []
    gt.append([0.0, 0.0, 0.0])
    for i in range(len(gps_topic)-1):
      x, y, z = get_xyz(gps_topic[i+1])
      x = x-x0
      y = y-y0
      z = z-z0
      gt.append([x, y, z])
      outfile.write("%.9F," % gps_topic[i+1, 0])
      outfile.write(str(gps_topic[i+1, 1]) + "," + str(gps_topic[i+1, 2]) + ",/gps,,")
      outfile.write("%.9F," % x)
      outfile.write("%.9F," % y)
      outfile.write("%.9F," % z)
      outfile.write("0.0,0.0,0.0,0.0")
      outfile.write("\n")
  gt = np.array(gt)

  # Init figure
  fig = pylab.figure(1)
  ax = Axes3D(fig)
  ax.grid(True)
  ax.set_title("GT Viewer")
  ax.set_xlabel("X")
  ax.set_ylabel("Y")
  ax.set_zlabel("Z")

  ax.plot(gt[:,0], gt[:,1], gt[:,2], 'g', label='Ground Truth')
  ax.legend()

  pyplot.draw()
  pylab.show()
