#include <pcl/io/pcd_io.h>
#include "Util/Viewer.h"
#include <pcl/filters/extract_indices.h>
using namespace std;
using namespace pcl;

// CONSTANTES
float BG_COLOR[]         = {0, 0, 0};
float POINTCLOUD_COLOR[] = {1.0, 1.0, 1.0};
float GRIPPER_COLOR[]    = {0, 0, 1};
float OBJECT_COLOR[]     = {1, 0, 0};
struct box{
	float center[3];
	float size[3];
};
typedef struct box Box;
vector<Box> gripper_boxes;
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
int main(int argc, char **argv){
	if (argc < 2){
		printf("Error: Debe especificar un archivo .pcd\n");
		exit(1);
	}
	// Inicializar cajas
	initBoxes();
	// Crear Viewer
	Viewer viewer = Viewer(BG_COLOR[0], BG_COLOR[1], BG_COLOR[2], 0.1);
	// Leer nube de puntos
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>());
	io::loadPCDFile(argv[1], *cloud);
	viewer.drawPointCloud(cloud, "_gripper_object_pc", POINTCLOUD_COLOR[0], POINTCLOUD_COLOR[1], POINTCLOUD_COLOR[2]);
	// Separar nube en objeto y gripper
	PointCloud<PointXYZ>::Ptr gripper_pc (new PointCloud<PointXYZ>()), object_pc(new PointCloud<PointXYZ>());
	isolateObject(cloud, gripper_pc, object_pc);
	viewer.drawPointCloud(gripper_pc, "gripper_pc", GRIPPER_COLOR[0], GRIPPER_COLOR[1], GRIPPER_COLOR[2]);
	viewer.drawPointCloud(object_pc, "object_pc", OBJECT_COLOR[0], OBJECT_COLOR[1], OBJECT_COLOR[2],3);
	// Dibujar cajas
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
	}
	viewer.show();

	return 0;
}
/*
	TODO:
		-Intentar hacer tamaño de boxes dependientes de la apertura del gripper

 */