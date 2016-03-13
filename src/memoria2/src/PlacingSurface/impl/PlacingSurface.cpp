#include "../PlacingSurface.h"

PlacingSurface::PlacingSurface(){}

void PlacingSurface::setCloud(PointCloud<PointXYZ>::Ptr new_cloud){
	cloud = new_cloud;
}
void PlacingSurface::setNormal(geometry_msgs::PoseStamped new_normal){
	normal = new_normal;
}
void PlacingSurface::setCentroid(geometry_msgs::PointStamped new_centroid){
	centroid = new_centroid;
}