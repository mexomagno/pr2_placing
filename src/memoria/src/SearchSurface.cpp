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
        cloud->points[i].z = rand()/RAND_MAX*factor*(int)cloud->points.size();
    }
    pointcloud = cloud;
}

bool callback(memoria::SearchSurface::Request& request, memoria::SearchSurface::Response& response){
    ROS_INFO("Request entrante");
    // Obtener Pointcloud
    PointCloud::Ptr pointcloud;
    getDummyCloud(pointcloud);
    memoria::ErrorMsg errormsg;
    pcl::toROSMsg(*pointcloud, response.pointcloud);
    
    // Crear código de error
    errormsg.retcode = 0;

    errormsg.what = errors[errormsg.retcode];
    response.error = errormsg;
    return true;
}
int main(int argc, char **argv){
    /*errors = {  "",
                "Superficie no encontrada",
                "Servicio 'LookAt' no disponible",
                "Error desconocido"};*/
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