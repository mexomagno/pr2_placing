/*
Servicio encargado de encontrar superficie de placing
Recibe: Nada
Retorna: - sensor_msgs/PointCloud2
         - memoria/ErrorMsg
*/

#include <string>
#include <vector>
#include <ros/ros.h>
#include <memoria/SearchSurface.h>
#include <sensor_msgs/PointCloud2.h>
#include <memoria/ErrorMsg.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

// CONSTANTES

// VARIABLES GLOBALES
sensor_msgs::PointCloud2 selected_surface;
// Códigos de error
vector<string> errors(4);

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void getDummyCloud(PointCloud::Ptr& pointcloud){
    PointCloud::Ptr cloud (new PointCloud);
    cloud->header.frame_id = "head_mount_kinect_ir_optical_frame";
    cloud->width = 20;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);
    for (size_t i = 0; i < cloud->points.size(); i++){
        double factor = 0.1;
        cloud->points[i].x = (i%5)*factor;
        cloud->points[i].y = floor(i/5)*factor;
        cloud->points[i].z = 1 + 1.0*(rand()/RAND_MAX)*factor;
    }
    ROS_INFO("Construí nube dummy de %d puntos", (int)cloud->size());
    pointcloud = cloud;
}
int searchSurface(){
    // Buscar superficie
    ROS_INFO("Buscando superficie");
    // Guardar superficie encontrada
    ROS_INFO("Encontrada. Guardando...");
    PointCloud::Ptr cloud (new PointCloud);
    getDummyCloud(cloud);
    pcl::toROSMsg(*cloud, selected_surface);
    ROS_INFO("Obtenida nube de %d puntos",(int)cloud->size());
    return 0;
}
bool callback(memoria::SearchSurface::Request& request, memoria::SearchSurface::Response& response){
    ROS_INFO("Request entrante");
    // Buscar superficie
    int retcode = searchSurface();
    // Almacenar superficie
    response.pointcloud = selected_surface;
    // Crear código de error
    memoria::ErrorMsg errormsg;
    errormsg.retcode = retcode;
    errormsg.what = errors[errormsg.retcode];
    response.error = errormsg;
    return true;
}
int main(int argc, char **argv){
    errors[0]="";
    errors[1]="Superficie no encontrada";
    errors[2]="Servicio 'LookAt' no disponible";
    errors[3]="Error desconocido";
    
    ros::init(argc, argv, "search_surface_server");
    ros::NodeHandle nh;
    // Crear servicio
    ros::ServiceServer service = nh.advertiseService("search_surface",callback);
    ROS_INFO("Listo para recibir requests");
    ros::spin();
    return 0;
}