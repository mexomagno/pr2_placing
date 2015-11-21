#include <vector>
#include <cmath>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h> // para isPointIn2DPolygon()
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/features/moment_of_inertia_estimation.h>

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
	// Centro de masa
	PointXYZ cm_;
	// Constantes
	static const double PI = 3.1416;
	static const double ANGLE_RELAXATION = 0.1; // para orientar normales

public:
	// Constructor
	Polymesh(PolygonMesh mesh);
	// --- Setters Getters ---
	void getCentroid(PointXYZ &p);
	void getBiggestPolygon(Vertices &polygon, int &poly_index, double &area, Eigen::Vector3f &normal, PointXYZ &centroid);
	void getBiggestFlatPatch(double angle_threshold, vector<int> &patch);
	PolygonMesh getPCLMesh();
	int getPolygonNumber();
	double getArea(int index);
	Eigen::Vector3f getNormal(int index);
	PointXYZ getCentroid(int index);

	// --- Utilidades ---
	PointXYZ projectPointOverPolygon(PointXYZ p, int poly_index);
	bool pointInPolygon(PointXYZ p, int poly_index);
	void flattenPatch(vector<int> patch, PointCloud<PointXYZ> &flatcloud);
private:
	bool triangleAreaNormalCentroid(PointXYZ p1, PointXYZ p2, PointXYZ p3, PointXYZ ptest, double &area, Eigen::Vector3f &normal, PointXYZ &centroid);
	int getAnyOtherIndex(int not_this);
	PointXYZ getCenterOfMass();
	double toGrad(double rads);
};