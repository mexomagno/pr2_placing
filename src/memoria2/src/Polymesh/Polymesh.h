#ifndef POLYMESH_H
#define POLYMESH_H

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
	// Centroide
	PointXYZ ct_;
	// Constantes
	static const double PI = 3.1416;
	static const double ANGLE_RELAXATION = 0.01; // para orientar normales

public:
	// Constructor
	Polymesh(PolygonMesh mesh);
	// --- Setters Getters ---
	PointXYZ computeCentroid();
	PointXYZ getMeshCentroid();
	void getBiggestPolygon(Vertices &polygon, int &poly_index, double &area, Eigen::Vector3f &normal, PointXYZ &centroid);
	void getBiggestFlatPatch(double angle_threshold, vector<int> &patch);
	void getFlatPatches(double angle_threshold, vector<vector<int> > &patch, vector<double> &areas);
	PolygonMesh getPCLMesh();
	PointCloud<PointXYZ>::Ptr getPointCloud();
	int getPolygonNumber();
	double getArea(int index);
	Eigen::Vector3f getNormal(int index);
	PointXYZ getCentroid(int index);
	PointXYZ getCenterOfMass();
	// --- Utilidades ---
	PointXYZ projectPointOverPolygon(PointXYZ p, int poly_index);
	static PointXYZ projectPointOverFlatPointCloud(PointXYZ p, PointCloud<PointXYZ>::Ptr cloud);
	bool pointInPolygon(PointXYZ p, int poly_index);
	void flattenPatch(vector<int> patch, PointCloud<PointXYZ> &flatcloud, ModelCoefficients::Ptr &patchcoefs);
	static bool isPointInConvexPolygon(PointXYZ p, PointCloud<PointXYZ> poly);
private:
	bool triangleAreaNormalCentroid(PointXYZ p1, PointXYZ p2, PointXYZ p3, double &area, Eigen::Vector3f &normal, PointXYZ &centroid);
	static Eigen::Vector3f triangleNormal(PointXYZ p1, PointXYZ p2, PointXYZ p3);
	int getAnyOtherIndex(int not_this);
	PointXYZ getCenterOfMassInternal();
	double toGrad(double rads);
};

#endif // POLYMESH_H
