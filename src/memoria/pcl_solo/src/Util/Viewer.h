#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <pcl/PolygonMesh.h>
#include "Polymesh.h"


using namespace std;
using namespace pcl;

class Viewer{
	/* variables internas */
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
	int n_text_; // Cantidad de textos añadidos. Para apilarlos sin sobreponerse.
	vector<double> visualizations_;

	/* --- Formato --- */
	static const int BOTTOM_MARGIN = 15;
	static const int LEFT_MARGIN   = 10;
	int FONT_SIZE;

public:
	// Constructor
	Viewer(float r, float g, float b);

	// Setea puntos - wireframe - surfaces
	void setVisualizationMode(const string shape_id, int mode);
	void setLineWidth(const string shape_id, int width);
	void setFontSize(const string shape_id, int size);
	void setColor(const string shape_id, float r, float g, float b, bool iscloud = false);
	// --- Métodos para dibujar --- //
	void drawPoint(PointXYZ p, const string shape_id, float r, float g, float b, float size);
	void drawPointCloud(PointCloud<PointXYZ>::Ptr cloud, const string shape_id, float r, float g, float b, int point_size = 1);
	void drawPolygonMesh(PolygonMesh mesh, const string shape_id, float r, float g, float b);
	void drawPolygon(Vertices polygon, PolygonMesh mesh, const string shape_id, float r, float g, float b, bool filled = true);
	void drawPolygonVector(vector<Vertices> polygon, PolygonMesh mesh, const string shape_id_prefix, float in_r, float in_g, float in_b, float out_r, float out_g, float out_b, int width);
	void drawPolygonMeshNormals(Polymesh mesh, const string shape_id, float r, float g, float b);
	void addText(const string text, const string shape_id, float r, float g, float b);
	void show();
};