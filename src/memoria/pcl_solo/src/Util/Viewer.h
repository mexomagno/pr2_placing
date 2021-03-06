#ifndef VIEWER_H
#define VIEWER_H

#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/PolygonMesh.h>
#include "Polymesh.h"


using namespace std;
using namespace pcl;

class Viewer{
	/* variables internas */
	int n_text_; // Cantidad de textos añadidos. Para apilarlos sin sobreponerse.
	vector<double> visualizations_;

	/* --- Formato --- */
	static const int BOTTOM_MARGIN = 15;
	static const int LEFT_MARGIN   = 10;
	int FONT_SIZE;
	static const float POINT_SIZE_PONDERATOR = 0.05;
	static const float NORMAL_SIZE_PONDERATOR = 0.15;

public:
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
	// Constructor
	Viewer(float r, float g, float b, float axisscale = 1.0);

	// Setea puntos - wireframe - surfaces
	void setVisualizationMode(const string shape_id, int mode);
	void setLineWidth(const string shape_id, int width);
	void setFontSize(const string shape_id, int size);
	void setColor(const string shape_id, float r, float g, float b, bool iscloud = false);
	// --- Métodos para dibujar --- //
	void drawPoint(PointXYZ p, const string shape_id, float r, float g, float b);
	void drawPoint(PointXYZ p, PointCloud<PointXYZ>::Ptr cloud, const string shape_id, float r, float g, float b);
	void drawPointCloud(PointCloud<PointXYZ>::Ptr cloud, const string shape_id, float r, float g, float b, int point_size = 1);
	void drawPolygonMesh(PolygonMesh mesh, const string shape_id, float r, float g, float b, float a = 1);
	void drawPolygon(Vertices polygon, PolygonMesh mesh, const string shape_id, float r, float g, float b, bool filled = true);
	void drawPolygonVector(vector<int> polygon, PolygonMesh mesh, const string shape_id_prefix, float in_r, float in_g, float in_b, float out_r, float out_g, float out_b, int width, float alpha = 1);
	void drawPolygonMeshNormals(Polymesh mesh, const string shape_id, float r, float g, float b);
	void drawBox(const string shape_id, float min_x, float max_x, float min_y, float max_y, float min_z, float max_z, float r, float g, float b, float a);
	void addText(const string text, const string shape_id, float r, float g, float b);
	void show();
};

#endif // VIEWER_H