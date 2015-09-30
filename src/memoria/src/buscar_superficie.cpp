/* 
Este algoritmo se basa en el flujo explicitado en "~/Escritorio/Memoria/Conceptos"
Básicamente es:
    -Dock Arms
    -Apuntar cabeza un poco al suelo
    -Buscar superficie:
        - Calzar modelo con RANSAC (plano primeramente)
        - if encontrado:
            - Aplicar restricciones
            - if cumple:
                return ENCONTRADO
        - NO ENCONTRADO
        if no encontrado:
            - buscar en otro lado
            - if busqué todo a mi alcance
                - return NO ENCONTRADO (ABORTAR)

Componentes necesarias:
LIBRERÍAS
    ROS
        ros/ros : para todo ros
        Mensajes:
            sensor_msgs/PointCloud2 : para recibir mensajes de kinect
            geometry_msgs/PointStamped : Para enviar requests al server de la cabeza
        memoria/LookAt :  Para usar el servicio lookat para control de la cabeza
        memoria/LookAtMsg :  Mensaje personalizado para el server de control de la cabeza
    PCL para procesamiento de nube de puntos del Kinect
        pcl/point_cloud : para usar clase pointcloud
        pcl/point_types : para usar tipos de puntos
        pcl/segmentation/sac_segmentation : para segmentacion
        pcl/filters/voxel_grid : Para submuestreo
        pcl/filters/extract_indices : para extracción de índices de inliers
        pcl/features/normal_3d : Orientación de vectores hacia viewpoint
        // pcl_conversions/pcl_conversions & 
        pcl_ros/point_cloud: para trabajar con tipos de datos de pcl directamente

    TF para transformaciones de frames
        tf/transform_listener : listener de transformaciones
    OTROS
        math : para operaciones matemáticas trigonométricas
        iostream : para usar <<
        string : Strings de más alto nivel que char *
    

*/
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <memoria/LookAt.h>
#include <memoria/LookAtMsg.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl_ros/point_cloud.h>
//#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <iostream>
#include <string>

// Ahorrarse el prefijo "std"
using namespace std;
// Acortar nombre del tipo pointcloud
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// CONSTANTES
const string KINECT_TOPIC = "head_mount_kinect/depth/points";
const string QUERY_TOPIC = "memoria/lookat";
const string WORLD_FRAME = "/odom_combined";
const string ROBOT_FRAME = "/base_footprint";
const double LEAFSIZEX = 0.05, LEAFSIZEY = 0.05, LEAFSIZEZ = 0.05; 

// VARIABLES GLOBALES
ros::Publisher aux_pointcloud_publisher;
memoria::LookAt lookat_srv;

// MÉTODOS
void kinectCallback(const PointCloud::ConstPtr& in_cloud){
    ROS_INFO("Nube recibida");

    // Submuestrear nube para procesamiento más rápido
    PointCloud::Ptr in_cloud_subsampled(new PointCloud), segmented(new PointCloud);
    pcl::VoxelGrid<pcl::PointXYZ> subsampler;
    subsampler.setInputCloud(in_cloud);
    subsampler.setLeafSize (LEAFSIZEX,LEAFSIZEY,LEAFSIZEZ);
    subsampler.filter(*in_cloud_subsampled);
    ROS_INFO("Subsampleo: %d puntos de %d (%f%%)\n",(int)in_cloud_subsampled->size(),(int)in_cloud->size(),(100.0*(int)in_cloud_subsampled->size()/(int)in_cloud->size()));
    aux_pointcloud_publisher.publish(in_cloud_subsampled);

    //
}

int main(int argc, char **argv){
    // Iniciar ROS
    ROS_INFO("Comenzando programa");
    ros::init(argc, argv, "memoria");
    ros::NodeHandle nh;
    // Suscribirse al Kinect
    ros::Subscriber kinect_sub = nh.subscribe<PointCloud>(KINECT_TOPIC, 1, kinectCallback);
    // "Subscribirse" al servicio de LookAt
    ros::ServiceClient lookat_client = nh.serviceClient<memoria::LookAt>("look_at");
    // Publicador auxiliar
    aux_pointcloud_publisher = nh.advertise<PointCloud> ("plano_submuestreado", 1);
    // Mirar un poco al suelo (por ahi por 5,0,0 respecto al robot)
    memoria::LookAtMsg lookatmsg;
    lookatmsg.frame_id = ROBOT_FRAME;
    lookatmsg.rotate = 0; // FALSE, no rotar, sino que mover al punto.
    lookatmsg.vector.x = 2; lookatmsg.vector.y = lookatmsg.vector.z = 0;
    lookat_srv.request.lookatmsg = lookatmsg;
    ROS_INFO("Mirando un poco hacia el suelo");
    if (lookat_client.call(lookat_srv)){
        ROS_INFO("OK");
    }
    else{
        ROS_ERROR("Error al llamar al servicio 'look_at'");
        return 1;
    }
    ros::spin();
    return EXIT_SUCCESS; //0
}