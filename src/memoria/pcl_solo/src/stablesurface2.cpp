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
float CM_COLOR[]                = {0.15, 0.8, 0.8};
float CT_COLOR[]                = {1.0, 0.0, 0.0};
float FLAT_SURFACE_COLOR[]      = {0.0, 1.0, 0.0};
float FLAT_SURFACE_WIRE_COLOR[] = {0.0, 0.0, 1.0};
float FLAT_SURFACE_ALPHA 		= 0.4;
float FLATTENED_PATCH_COLOR []  = {1.0, 0.0, 0.0};
int FLAT_SURFACE_WIRE_WIDTH     = 3;
float LIGHT_FACTOR              = 0.7;
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
	viewer.drawPolygonMesh(mesh.getPCLMesh(), "Object ConvexHull", CONVEXHULL_COLOR[0], CONVEXHULL_COLOR[1], CONVEXHULL_COLOR[2], CONVEXHULL_ALPHA);

	// Visualizar normales
	//viewer.drawPolygonMeshNormals(mesh, "ConvexHull Normals", NORMALS_COLOR[0], NORMALS_COLOR[1], NORMALS_COLOR[2]);

	// Obtener listado de parches y de sus respectivas áreas
	vector<vector<int> > patches; // todos los parches posibles, ORDENADOS desde el más grande al más pequeño
	vector<double> patches_areas; // sus areas correspondientes, en el orden adecuado
	mesh.getFlatPatches(PATCH_ANGLE_THRESHOLD, patches, patches_areas);
	// Obtener centro de masa
	PointXYZ cm = mesh.getCenterOfMass();

	// Iterar hasta encontrar un parche estable. Priorizar parches grandes
	vector<int> best_patch;
	double best_patch_area;
	PointCloud<PointXYZ>::Ptr patch_plane(new PointCloud<PointXYZ>());
	PointXYZ cm_proj;
	bool plane_found = false;
	for (int i=0; i<patches.size(); i++){
		// Obtener plano representado por el parche
		mesh.flattenPatch(patches[i], *patch_plane);
		// proyectar centro de masa sobre plano
		cm_proj = Polymesh::projectPointOverFlatPointCloud(cm, patch_plane);
		// Verificar si proyección está dentro del plano del parche
		if (isPointIn2DPolygon(cm_proj, *patch_plane)){
			best_patch = patches[i];
			best_patch_area = patches_areas[i];
			printf("Se ha encontrado un plano estable (de area %f)\n", best_patch_area);
			viewer.addText("Posicion estable encontrada", "posicion_estable_label", 0, 1, 0);
			plane_found = true;
			break;
		}
	}
	if (not plane_found){
		printf("No se ha podido encontrar un plano estable. Qué situación más rara!!\n");
		exit(0);
	}
	viewer.drawPolygonVector(best_patch, mesh.getPCLMesh(), "flat_patch", FLAT_SURFACE_COLOR[0], FLAT_SURFACE_COLOR[1], FLAT_SURFACE_COLOR[2], FLAT_SURFACE_WIRE_COLOR[0], FLAT_SURFACE_WIRE_COLOR[1], FLAT_SURFACE_WIRE_COLOR[2], FLAT_SURFACE_WIRE_WIDTH, FLAT_SURFACE_ALPHA);
	viewer.drawPointCloud(patch_plane, "Flattened Patch", 1, 0, 0, 7);
	viewer.drawPoint(cm, mesh.getPointCloud(), "Center of Mass", CM_COLOR[0], CM_COLOR[1], CM_COLOR[2]);
	viewer.drawPoint(cm_proj, mesh.getPointCloud(), "Center of Mass projected", CM_COLOR[0]+(1-CM_COLOR[0])*LIGHT_FACTOR, CM_COLOR[1]+(1-CM_COLOR[1])*LIGHT_FACTOR, CM_COLOR[2]+(1-CM_COLOR[2])*LIGHT_FACTOR);
	viewer.addText("Centro de Masa proyectado", "centro_de_masa_proy_label", CM_COLOR[0]+(1-CM_COLOR[0])*LIGHT_FACTOR, CM_COLOR[1]+(1-CM_COLOR[1])*LIGHT_FACTOR, CM_COLOR[2]+(1-CM_COLOR[2])*LIGHT_FACTOR);
	viewer.addText("Centro de Masa", "centro_de_masa_label", CM_COLOR[0], CM_COLOR[1], CM_COLOR[2]);
	viewer.show();
	return 0;
}

/* 
TODO:
	[DONE]- Reparar problema de normales: Algunas quedan invertidas (las menos!)
		ideas: 
			* Enderezar normales, luego filtrar el disconexo
			* Revisar condición de ángulo entre normal y delta: Pedir otro punto test si el angulo es muy cercano a PI/2 
				Se implementó esto y no parece mejorar mucho.
			[implementado]* Orientar según centroide
	[DONE]- Proyectar centro de masa sobre área encerrada por el parche
	[DONE]- Reparar tamaño de parche aplanado
		Al parecer es puramente un problema del convex hull. Sin embargo la proyección se hace sobre los puntos, que están correctos.
	[DONE]- Iterar parches si el seleccionado no es estable
	- Revisar problema de isPointIn2DPolygon (ver "pcds/convertidos/trofeo.pcd" 0.01
	- Aplicar optimizaciones al algoritmo de parches
	- Filtrar parches según posición del gripper
		- ¿Priorizar parches lejanos al gripper primero?
	
*/