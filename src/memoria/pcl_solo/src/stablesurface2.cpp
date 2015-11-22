#include <pcl/io/pcd_io.h>
#include "Util/Util.h"
//#include "Util/Polymesh.h"
#include "Util/Viewer.h"

using namespace std;
using namespace pcl;
// CONSTANTES
/* --- Formatos de visualización --- */
float BG_COLOR[]                = {0.0, 0.0, 0.0};
float POINTCLOUD_COLOR[]        = {1.0, 1.0, 1.0};
float CONVEXHULL_COLOR[]        = {0.7, 0.7, 0.7};
float CONVEXHULL_ALPHA 			= 0.2;
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
//	viewer.drawPolygonMesh(mesh.getPCLMesh(), "Object ConvexHull", CONVEXHULL_COLOR[0], CONVEXHULL_COLOR[1], CONVEXHULL_COLOR[2], CONVEXHULL_ALPHA);
	viewer.drawPolygonMesh(mesh.getPCLMesh(), "Object ConvexHull", CONVEXHULL_COLOR[0], CONVEXHULL_COLOR[1], CONVEXHULL_COLOR[2]);

	// Visualizar normales
	viewer.drawPolygonMeshNormals(mesh, "ConvexHull Normals", NORMALS_COLOR[0], NORMALS_COLOR[1], NORMALS_COLOR[2]);

	// Obtener parche más grande, convertirlo a polygonmesh y visualizarlo
	vector<int> patch; // Indices a los polígonos del mesh
	mesh.getBiggestFlatPatch(PATCH_ANGLE_THRESHOLD, patch);
	printf("Se obtuvo parche de %d polígonos\n", (int)patch.size());
	viewer.drawPolygonVector(patch, mesh.getPCLMesh(), "flat_patch", FLAT_SURFACE_COLOR[0], FLAT_SURFACE_COLOR[1], FLAT_SURFACE_COLOR[2], FLAT_SURFACE_WIRE_COLOR[0], FLAT_SURFACE_WIRE_COLOR[1], FLAT_SURFACE_WIRE_COLOR[2], FLAT_SURFACE_WIRE_WIDTH);
	// Obtener "sombra" del parche más grande y visualizarlo
	PointCloud<PointXYZ>::Ptr patch_flat(new PointCloud<PointXYZ>());
	mesh.flattenPatch(patch, *patch_flat);
	// Obtener polygonmesh del parche plano. Quizás debiera ser retornada directamente por flattenPatch().
	PolygonMesh patch_mesh = Util::getConvexHull(patch_flat);
//	viewer.drawPolygonMesh(patch_mesh, "flat_patch_mesh", 0, 1, 1);

	// Obtener centro de masa, proyectarlo en sombra del parche y mostrarlo
	PointXYZ cm = mesh.getCenterOfMass();
	PointXYZ cm_proj = Polymesh::projectPointOverFlatPointCloud(cm, patch_flat);
//	viewer.drawPoint(cm_proj, mesh.getPointCloud(), "Center of Mass projected", CM_COLOR[0], CM_COLOR[1], CM_COLOR[2]);

	// Mostrar centroide de toda la mesh
	viewer.drawPoint(mesh.getMeshCentroid(), mesh.getPointCloud(), "Centroid", CT_COLOR[0], CT_COLOR[1], CT_COLOR[2]);
	// Decidir si está dentro del parche plano o no
	bool cm_in_patch = isPointIn2DPolygon(cm_proj, *patch_flat);
	printf("Plano %s es estable: centro de masa %s en plano\n",(cm_in_patch ? "si" : "NO"), (cm_in_patch ? "se proyecta" : "NO SE PROYECTA"));
//	if (cm_in_patch)
//		viewer.addText("Posicion estable", "stable_position_label", 0, 1, 0);
//	else
//		viewer.addText("POSICION INESTABLE", "stable_position_label", 1, 0, 0);
	viewer.show();
	return 0;
}

/* 
TODO:
	- Reparar problema de normales: Algunas quedan invertidas (las menos!)
		ideas: 
			* Enderezar normales, luego filtrar el disconexo
			* Revisar condición de ángulo entre normal y delta: Pedir otro punto test si el angulo es muy cercano a PI/2 
				Se implementó esto y no parece mejorar mucho.
	[DONE]- Proyectar centro de masa sobre área encerrada por el parche
	- Aplicar optimizaciones al algoritmo de parches
	- Filtrar parches según posición del gripper
		- ¿Priorizar parches lejanos al gripper primero?

*/