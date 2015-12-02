#include <pcl/io/pcd_io.h>
#include "Util/Util.h"
//#include "Util/Polymesh.h"
#include "Util/Viewer.h"
#include <pcl/filters/extract_indices.h>

using namespace std;
using namespace pcl;
// CONSTANTES
/* --- Formatos de visualización --- */
float BG_COLOR[]                = {0.0, 0.0, 0.0};
float POINTCLOUD_COLOR[]        = {1.0, 1.0, 1.0};
float GRIPPER_COLOR[]           = {0, 0, 1};
float OBJECT_COLOR[]            = {1, 0, 0};
float CONVEXHULL_COLOR[]        = {0.7, 0.7, 0.7};
float CONVEXHULL_ALPHA 			      = 0.2;
int NORMALS_COLOR[]             = {1.0, 0.0, 1.0};
float CM_COLOR[]                = {0.15, 0.8, 0.8};
float CT_COLOR[]                = {1.0, 0.0, 0.0};
float FLAT_SURFACE_COLOR[]      = {0.0, 1.0, 0.0};
float FLAT_SURFACE_WIRE_COLOR[] = {0.0, 0.0, 1.0};
float FLAT_SURFACE_ALPHA 		     = 0.4;
float FLATTENED_PATCH_COLOR []  = {1.0, 0.0, 0.0};
int FLAT_SURFACE_WIRE_WIDTH     = 3;
float LIGHT_FACTOR              = 0.7;
float DARK_FACTOR               = 0.5;
/* --- Constantes para el algoritmo --- */
double PATCH_ANGLE_THRESHOLD    = 0.2;

// VARIABLES GLOBALES
struct box{
	float center[3];
	float size[3];
};
typedef struct box Box;
vector<Box> gripper_boxes;

// METODOS

void initBoxes(){
	// Base del gripper
	Box box1;
	box1.center[0] = -0.095; box1.center[1] = box1.center[2] = 0;
	box1.size[0] = 0.098; box1.size[1] = 0.16; box1.size[2] = 0.061;
	gripper_boxes.push_back(box1);
	// Dedos del gripper
	Box box2;
	box2.center[0] = -0.03; box2.center[1] = box2.center[2] = 0;
	box2.size[0] = 0.101; box2.size[1] = 0.18; box2.size[2] = 0.03;
	gripper_boxes.push_back(box2);
}
/**
 * Toma punto y retorna True si está dentro de caja (definida globalmente) y false en caso contrario.
 * @param p: PointXYZ con punto
 * @return: true, false.
 */
bool isPointInsideBox(PointXYZ p, struct box box){
	bool in_x = (p.x > box.center[0] - box.size[0]/2.0) and (p.x < box.center[0] + box.size[0]/2.0);
	bool in_y = (p.y > box.center[1] - box.size[1]/2.0) and (p.y < box.center[1] + box.size[1]/2.0);
	bool in_z = (p.z > box.center[2] - box.size[2]/2.0) and (p.z < box.center[2] + box.size[2]/2.0);
	return in_x and in_y and in_z;
}
/**
	Toma nube de puntos con gripper y objeto tomado, y los separa, entregando dos nubes diferentes.
	La forma de separarlos es considerando al gripper como un paralelepípedo y luego:
		- Obtener todos los puntos dentro del paralelepípedo
		- Acumular sus índices
		- Retornar puntos dentro y fuera.

	@param cloud_in: Nube con scaneo
	@param gripper_out: Referencia de nube donde retornar gripper aislado
	@param object_out: Referencia de nube donde retornar objeto aislado
	@return: Nada.
 */
void isolateObject(PointCloud<PointXYZ>::Ptr cloud_in, PointCloud<PointXYZ>::Ptr &gripper_out, PointCloud<PointXYZ>::Ptr &object_out){
	// Capturar índices de puntos dentro de paralelepípedo
	PointIndices::Ptr inliers (new PointIndices());
	for (int i=0; i < cloud_in->points.size(); i++){
		for (int j=0; j < gripper_boxes.size(); j++){
			if (isPointInsideBox(cloud_in->points[i], gripper_boxes[j])){
				inliers->indices.push_back(i);
				break;
			}
		}
	}
	// Extraer índices de cada figura
	ExtractIndices<PointXYZ> extractor;
	extractor.setInputCloud(cloud_in);
	extractor.setIndices(inliers);
	extractor.setNegative(false);
	extractor.filter(*gripper_out);
	extractor.setNegative(true);
	extractor.filter(*object_out);
}
/**
 * Verifica si un plano corta a una nube en dos
 * @param  pc    nube de puntos a evaluar
 * @param  coefs Coeficientes del plano
 * @return       True si lo corta, false en caso contrario.
 */
bool isPointCloudCutByPlane(PointCloud<PointXYZ> pc, ModelCoefficients::Ptr coefs){
	
}

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
	Viewer viewer = Viewer(BG_COLOR[0], BG_COLOR[1], BG_COLOR[2], 0.1);
	
	// Añadir texto de threshold
	stringstream thres_s;
	thres_s << "Threshold: " << PATCH_ANGLE_THRESHOLD;
	viewer.addText(thres_s.str(), "Threshold label", 1, 1, 1);

	// Cargar nube de puntos y visualizar
	PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>());
	io::loadPCDFile(argv[1], *cloud);
	viewer.drawPointCloud(cloud, "Object PointCloud", POINTCLOUD_COLOR[0], POINTCLOUD_COLOR[1], POINTCLOUD_COLOR[2]);

	// Separar objeto del gripper
	initBoxes();
	PointCloud<PointXYZ>::Ptr gripper_pc (new PointCloud<PointXYZ>()), object_pc(new PointCloud<PointXYZ>());
	isolateObject(cloud, gripper_pc, object_pc);
	viewer.drawPointCloud(gripper_pc, "gripper_pc", GRIPPER_COLOR[0], GRIPPER_COLOR[1], GRIPPER_COLOR[2]);
	viewer.drawPointCloud(object_pc, "object_pc", OBJECT_COLOR[0], OBJECT_COLOR[1], OBJECT_COLOR[2],3);
	/*// Dibujar cajas
	for (int i=0; i<gripper_boxes.size(); i++){
		stringstream box_name;
		box_name << "box_" << i;
		Box thisbox = gripper_boxes[i];
		float min_x = thisbox.center[0] - thisbox.size[0]/2.0;
		float max_x = thisbox.center[0] + thisbox.size[0]/2.0;
		float min_y = thisbox.center[1] - thisbox.size[1]/2.0;
		float max_y = thisbox.center[1] + thisbox.size[1]/2.0;
		float min_z = thisbox.center[2] - thisbox.size[2]/2.0;
		float max_z = thisbox.center[2] + thisbox.size[2]/2.0;
		viewer.drawBox(box_name.str(), min_x, max_x, min_y, max_y, min_z, max_z, 1, 1, 1, 0.5);
	}*/

	// Crear convex hull del objeto y visualizar
	Polymesh mesh = Polymesh(Util::getConvexHull(object_pc));
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
	ModelCoefficients::Ptr patch_plane_coefs(new ModelCoefficients());
	PointXYZ cm_proj;
	bool plane_found = false;
	for (int i=0; i<patches.size(); i++){
		// Obtener plano representado por el parche
		mesh.flattenPatch(patches[i], *patch_plane, patch_plane_coefs);
		// proyectar centro de masa sobre plano
		cm_proj = Polymesh::projectPointOverFlatPointCloud(cm, patch_plane);
		// VERIFICACIÓN DE CONDICIONES:
		// 		- Es un plano estable?
		// 			* centro de masa se proyecta sobre parche?
		// 		- Puede el gripper llegar a esa posición?
		// 			* El gripper es cortado por el plano?
		if (isPointIn2DPolygon(cm_proj, *patch_plane)){
			best_patch = patches[i];
			best_patch_area = patches_areas[i];
			printf("Se ha encontrado un plano estable (de area %.2f)\n", best_patch_area);
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
	- Limitar plano según alcance del gripper
	- Intentar hacer tamaño de boxes dependientes de la apertura del gripper
	- Evaluar agregar restricción de orientación para cuencos. 
	
*/