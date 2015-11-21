// #include <iostream>
// #include <sstream>
// #include <vector>
// #include <cmath>
// #include <boost/thread/thread.hpp>
// #include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/ModelCoefficients.h>
// #include <pcl/segmentation/extract_polygonal_prism_data.h> // para isPointIn2DPolygon()
// #include <pcl/surface/convex_hull.h>
// #include <pcl/PolygonMesh.h>
// #include <pcl/filters/project_inliers.h>
#include "Util/Util.h"
#include "Util/Viewer.h"
#include "Util/Polymesh.h"

using namespace pcl,std;
typedef PointCloud<PointXYZ> PointCloud;
// CONSTANTES
/* --- Formatos de visualización --- */
float BG_COLOR[]                = {0.0, 0.0, 0.0};
float POINTCLOUD_COLOR[]		= {1.0, 1.0, 1.0};
float CONVEXHULL_COLOR[]		= {0.7, 0.7, 0.7};
int NORMALS_COLOR[]             = {1.0, 0.0, 1.0};
float CM_COLOR[]                = {1.0, 0.5, 0.0};
float CT_COLOR[]                = {1.0, 0.0, 0.0};
float FLAT_SURFACE_COLOR[]      = {0.0, 1.0, 0.0};
float FLAT_SURFACE_WIRE_COLOR[] = {0.0, 0.0, 1.0};
float FLAT_SURFACE_WIRE_WIDTH   = 3;
float LIGHT_FACTOR              = 0.5;
float DARK_FACTOR               = 0.5;
/* --- Constantes para el algoritmo --- */
double PATCH_ANGLE_THRESHOLD = 0.2;
double PI                    = 3.1415;

// VARIABLES GLOBALES
Viewer viewer;

int main(int argc, char** argv){
	// Obtener argumentos: Archivo pcd y opcionalmente, angle_threshold
	if (argc < 2){
		printf("Error: Debe especificar un archivo .pcd\n");
		exit(1);
	}
	if (argc == 3){
		PATCH_ANGLE_THRESHOLD = atof(argv[2]);
	}

	// Cargar nube de puntos
	PointCloud::Ptr cloud (new PointCloud());
	io::loadPCDFile(argv[1], *cloud);
	// Crear visualizador, ver nube de puntos
	viewer = Viewer(BG_COLOR[0], BG_COLOR[1], BG_COLOR[2]);
	viewer.drawPointCloud(cloud, "Object PointCloud", POINTCLOUD_COLOR[0], POINTCLOUD_COLOR[1], POINTCLOUD_COLOR[2]);
	// Crear convex hull y visualizar
	PolyMesh mesh = Polymesh(Util.getConvexHull(cloud));
	viewer.drawPolygonMesh(mesh.getPCLMesh(), "Object ConvexHull", CONVEXHULL_COLOR[0], CONVEXHULL_COLOR[1], CONVEXHULL_COLOR[2]);
	// Visualizar normales
	viewer.drawPolygonMeshNormals(mesh.getPCLMesh(), "ConvexHull Normals", NORMALS_COLOR[0], NORMALS_COLOR[1], NORMALS_COLOR[2]);
	viewer.show();
	return 0;
}