#ifndef KINECT_H
#define KINECT_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include "../../Util/Util.h"

using namespace std;
using namespace pcl;

class Kinect{
	ros::NodeHandle *nh_;
	ros::Subscriber cloud_sub_;
	PointCloud<PointXYZ>::Ptr last_cloud_;
	public:
		// Constructor y Destructor
		Kinect();
		~Kinect();
		// Métodos
		PointCloud<PointXYZ>::Ptr getNewCloud();
	private:

};


#endif // KINECT_H