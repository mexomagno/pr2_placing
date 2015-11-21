#include "Util.h"

PointXYZ Util::getCentroid(PointCloud<PointXYZ> pc){
	int pc_size = pc.points.size();
	Eigen::Vector3f accum;
	for (int i=0; i<pc_size; i++){
		accum += Eigen::Vector3f(pc.points[i].x, pc.points[i].y, pc.points[i].z);
	}
	accum /= pc_size;
	return PointXYZ(accum[0], accum[1], accum[2]);
}
PolygonMesh Util::getConvexHull(PointCloud<PointXYZ>::Ptr cloud){
	PolygonMesh mesh;
    ConvexHull<pcl::PointXYZ> chull;
    chull.setInputCloud(cloud);
    chull.reconstruct(mesh);
    return mesh;
}
