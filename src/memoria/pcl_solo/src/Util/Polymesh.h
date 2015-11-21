#include <vector>
#include <cmath>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h> // para isPointIn2DPolygon()

using namespace std;
using namespace pcl;

class Polymesh{
	// PolygonMesh pcl
	PolygonMesh mesh_;
	// Cantidad de polígonos
	int poly_number_;
	// PointCloud utilizable
	PointCloud<PointXYZ>::Ptr meshcloud_;
	// Lista de areas de polígonos
	vector<double> poly_areas_;
	// Lista de normales de polígonos
	vector<Eigen::Vector3f> poly_normals_;
	// Lista de centroides de polígonos
	vector<PointXYZ> poly_centroids_;

	// Constantes
	static const double PI = 3.1415;

public:
	// Constructor
	Polymesh(PolygonMesh mesh);
	// --- Setters Getters ---
	void getCentroid(PointXYZ &p);
	void getCenterOfMass(PointXYZ &p);
	void getBiggestPolygon(Vertices &polygon, int &poly_index, double &area, Eigen::Vector3f &normal, PointXYZ &centroid);
	void getBiggestFlatPatch(double angle_threshold, vector<Vertices> &patch);
	PolygonMesh getPCLMesh();
	int getPolygonNumber();
	double getArea(int index);
	Eigen::Vector3f getNormal(int index);
	PointXYZ getCentroid(int index);
	PointXYZ projectPointOverPolygon(PointXYZ p, int poly_index);
	bool pointInPolygon(PointXYZ p, int poly_index);
private:
	void triangleAreaNormalCentroid(PointXYZ p1, PointXYZ p2, PointXYZ p3, PointXYZ ptest, double &area, Eigen::Vector3f &normal, PointXYZ &centroid);
	int getAnyOtherIndex(int not_this);
};