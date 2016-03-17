#ifndef PLACEDOBJECT_H
#define PLACEDOBJECT_H


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include "../Polymesh/Polymesh.h"
#include "../Util/Util.h"

using namespace std;
using namespace pcl;

#ifndef STRUCT_BOX
#define STRUCT_BOX
struct box{
    float center[3];
    float size[3];
};
typedef struct box Box;
#endif // STRUCT_BOX
		
#ifndef STRUCT_BOUNDING_BOX
#define STRUCT_BOUNDING_BOX
struct bounding_box{
	PointXYZ min;
	PointXYZ max;
	geometry_msgs::Point position;
    geometry_msgs::Quaternion rotation;
};
typedef struct bounding_box BBOriented;
#endif // STRUCT_BOUNDING_BOX

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
	// bounding box
	BBOriented bounding_box;
	// Mesh
	Polymesh polymesh;

	// MÉTODOS
	PlacedObject();
	// setters
	void setCloud(PointCloud<PointXYZ>::Ptr object_cloud);
	void setBaseArea(float new_base_area);
	// void setBoundingBox(vector<PointXYZ> new_bounding_box);
	// void setStablePose(geometry_msgs::PoseStamped stable_pose);

	bool computeStablePose();
private:
	void computeFeatures();
};



#endif // PLACEDOBJECT_H