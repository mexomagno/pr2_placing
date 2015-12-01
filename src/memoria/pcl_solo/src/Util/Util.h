#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/surface/convex_hull.h>

using namespace std;
using namespace pcl;

class Util{
public:
	static PointXYZ getCentroid(PointCloud<PointXYZ> pc);
	static PolygonMesh getConvexHull(PointCloud<PointXYZ>::Ptr cloud);
};