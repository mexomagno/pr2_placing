#include <pcl/io/pcd_io.h>
#include "Util/Util.h"
#include "Util/Viewer.h"

using namespace std;
using namespace pcl;
// CONSTANTES
/* --- Formatos de visualización --- */
float BG_COLOR[]                = {0.0, 0.0, 0.0};
float POINTCLOUD_COLOR[]        = {1.0, 1.0, 1.0};
float CONVEXHULL_COLOR[]        = {0.7, 0.7, 0.7};
int NORMALS_COLOR[]             = {1.0, 0.0, 1.0};
float CM_COLOR[]                = {1.0, 0.5, 0.0};
float CT_COLOR[]                = {1.0, 0.0, 0.0};
float FLAT_SURFACE_COLOR[]      = {0.0, 1.0, 0.0};
float FLAT_SURFACE_WIRE_COLOR[] = {0.0, 0.0, 1.0};
float FLATTENED_PATCH_COLOR []  = {1.0, 0.0, 0.0};
int FLAT_SURFACE_WIRE_WIDTH     = 3;
float LIGHT_FACTOR              = 0.5;
float DARK_FACTOR               = 0.5;
/* --- Constantes para el algoritmo --- */
double PATCH_ANGLE_THRESHOLD    = 0.2;

// VARIABLES GLOBALES


int main(int argc, char** argv){
	// Obtener argumentos: Archivo pcd y opcionalmente, angle_threshold
	if (argc < 2){
		printf("Error: Debe especificar un archivo .pcd\n");
		exit(1);
	}
	if (argc == 3){
		PATCH_ANGLE_THRESHOLD = atof(argv[2]);
	}
	// Crear visualizador
	Viewer viewer = Viewer(BG_COLOR[0], BG_COLOR[1], BG_COLOR[2]);
	
	// Añadir texto de threshold
	stringstream thres_s;
	thres_s << "Threshold: " << PATCH_ANGLE_THRESHOLD;
	viewer.addText(thres_s.str(), "Threshold label", 1, 1, 1);

	// Cargar nube de puntos y visualizar
	PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>());
	io::loadPCDFile(argv[1], *cloud);
	viewer.drawPointCloud(cloud, "Object PointCloud", POINTCLOUD_COLOR[0], POINTCLOUD_COLOR[1], POINTCLOUD_COLOR[2]);

	// Crear convex hull y visualizar
	Polymesh mesh = Polymesh(Util::getConvexHull(cloud));
	viewer.drawPolygonMesh(mesh.getPCLMesh(), "Object ConvexHull", CONVEXHULL_COLOR[0], CONVEXHULL_COLOR[1], CONVEXHULL_COLOR[2]);

	// Visualizar normales
	viewer.drawPolygonMeshNormals(mesh, "ConvexHull Normals", NORMALS_COLOR[0], NORMALS_COLOR[1], NORMALS_COLOR[2]);

	// Obtener parche más grande y visualizarlo
	vector<int> patch; // Indices a los polígonos del mesh
	mesh.getBiggestFlatPatch(PATCH_ANGLE_THRESHOLD, patch);
	printf("Se obtuvo parche de %d polígonos\n", (int)patch.size());
	viewer.drawPolygonVector(patch, mesh.getPCLMesh(), "flat_patch", FLAT_SURFACE_COLOR[0], FLAT_SURFACE_COLOR[1], FLAT_SURFACE_COLOR[2], FLAT_SURFACE_WIRE_COLOR[0], FLAT_SURFACE_WIRE_COLOR[1], FLAT_SURFACE_WIRE_COLOR[2], FLAT_SURFACE_WIRE_WIDTH);
	// Obtener "sombra" del parche más grande y visualizarlo
	PointCloud<PointXYZ>::Ptr patch_flat(new PointCloud<PointXYZ>());
	mesh.flattenPatch(patch, *patch_flat);
	viewer.drawPointCloud(patch_flat, "Parche aplanado", FLATTENED_PATCH_COLOR[0], FLATTENED_PATCH_COLOR[1], FLATTENED_PATCH_COLOR[2], 10);

	viewer.show();
	return 0;
}

/* 
TODO:
	[DONE]- Reparar problema de normales: Algunas quedan invertidas (las menos!)
		ideas: 
			* Enderezar normales, luego filtrar el disconexo
			* Revisar condición de ángulo entre normal y delta: Pedir otro punto test si el angulo es muy cercano a PI/2 
	- Proyectar centro de masa sobre área encerrada por el parche
	- Aplicar optimizaciones al algoritmo de parches
	- Filtrar parches según posición del gripper
	- ¿Priorizar parches lejanos al gripper primero?

*/