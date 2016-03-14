#ifndef PLACEDOBJECT_H
#define PLACEDOBJECT_H


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "../Polymesh/Polymesh.h"
#include "../Util/Util.h"

using namespace std;
using namespace pcl;

class PlacedObject{
	// VARIABLES INTERNAS
		
public:
	// VARIABLES
	// nube de puntos objeto
	PointCloud<PointXYZ>::Ptr object_pc;
	// nube de puntos gripper
	PointCloud<PointXYZ>::Ptr gripper_pc;
	// pose estable
	geometry_msgs::PoseStamped stable_pose;
	// area apoyo
	float base_area;

	// MÉTODOS
	PlacedObject();
	// setters
	void setClouds(PointCloud<PointXYZ>::Ptr object_cloud, PointCloud<PointXYZ>::Ptr gripper_cloud);
	void setBaseArea(float new_base_area);
	// void setStablePose(geometry_msgs::PoseStamped stable_pose);

	bool computeStablePose();
private:
};



#endif // PLACEDOBJECT_H