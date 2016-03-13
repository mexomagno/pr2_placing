#ifndef PLACINGSURFACE_H
#define PLACINGSURFACE_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include "../Polymesh/Polymesh.h"
// #include "../Util/Util.h"

using namespace std;
using namespace pcl;

class PlacingSurface{
	// VARIABLES INTERNAS
	// 

public:
	// VARIABLES
	// nube de puntos
	PointCloud<PointXYZ>::Ptr cloud;
	// Vector normal
	geometry_msgs::PoseStamped normal;
	// Centroide
	geometry_msgs::PointStamped centroid;

	// MÉTODOS
	PlacingSurface();
	// setters
	void setCloud(PointCloud<PointXYZ>::Ptr new_cloud);
	void setNormal(geometry_msgs::PoseStamped new_normal);
	void setCentroid(geometry_msgs::PointStamped new_centroid);
private:

};



#endif // PLACINGSURFACE_H