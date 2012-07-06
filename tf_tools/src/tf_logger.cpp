#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <string>
#include <iostream>
#include <fstream>
#include <cmath>

using namespace std;

ostream& operator<< (ostream& os, const tf::Quaternion& quat)
{
	os << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w();
	return os;
}

ostream& operator<< (ostream& os, const tf::Vector3& trans)
{
	os << trans.x() << " " << trans.y() << " " << trans.z();
	return os;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tf_logger");

	ros::NodeHandle node("~");
	
	string baseLinkFrame;
	node.param<string>("baseLinkFrame", baseLinkFrame, "/base_link");
	string odomFrame;
	node.param<string>("odomFrame", odomFrame, "/odom");
	string kinectFrame;
	node.param<string>("kinectFrame", kinectFrame, "/openni_rgb_optical_frame");
	string worldFrame;
	node.param<string>("worldFrame", worldFrame, "/world");
	string outputFileName;
	node.param<string>("outputFileName", outputFileName, "output.txt");
	cout << baseLinkFrame << " " << odomFrame << " " << kinectFrame << " " <<
			worldFrame << " " << outputFileName << endl;

	tf::TransformListener t(ros::Duration(20));
	tf::StampedTransform tr_o, tr_i;
	
	ROS_INFO_STREAM("waiting for initial transforms");
	while (node.ok())
	{
		ros::Time now(ros::Time::now());
		if (t.waitForTransform(baseLinkFrame, now, odomFrame, now, odomFrame, ros::Duration(1)))
			break;
	}
	ROS_INFO_STREAM("got first odom to baseLink");
	while (node.ok())
	{
		ros::Time now(ros::Time::now());
		if (t.waitForTransform(kinectFrame, now, worldFrame, now, worldFrame, ros::Duration(1)))
			break;
	}
	ROS_INFO_STREAM("got first world to kinect");
	
	sleep(3);
	
	ros::Rate rate(10);
	ofstream ofs(outputFileName.c_str());
	while (node.ok())
	{
		// sleep
		ros::spinOnce();
		rate.sleep();
		
		// get parameters from transforms
		ros::Time curTime(ros::Time::now());
		if (!t.waitForTransform(baseLinkFrame, odomFrame, curTime, ros::Duration(3)))
			break;
		if (!t.waitForTransform(kinectFrame, worldFrame, curTime, ros::Duration(3)))
			break;
		ofs << curTime << " ";
		t.lookupTransform(baseLinkFrame, odomFrame, curTime, tr_o);
		ofs << tr_o.getOrigin() << " " << tr_o.getRotation() << " ";
		t.lookupTransform(kinectFrame, worldFrame, curTime,  tr_i);
		ofs << tr_i.getOrigin() << " " << tr_i.getRotation() << endl;
	}
	
	return 0;
}
