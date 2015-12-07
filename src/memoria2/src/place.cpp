/*
 Este es el nodo principal de la memoria.
 Este programa se preocupa de hacer todo lo necesario para lograr que el robot posicione el objeto
 */
#include <string>
#include <csignal>
#include <ros/ros.h>
#include <ros/console.h> // Para debuggear
// Mensajes
#include <geometry_msgs/PoseStamped.h>
// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
// Propios
#include "Util/Util.h"
#include "RobotDriver/RobotDriver.h"


using namespace std;
using namespace pcl;

// VARIABLES GLOBALES
RobotDriver *r_driver;
char grasp_arm;
void endProgram(int retcode){
    ROS_INFO("Terminando programa...");
    ros::shutdown();
    delete r_driver;
    exit(retcode);
}
void signalHandler( int signum ){
    ROS_INFO("Se recibe Ctrl+C");
    endProgram(0);
}
bool searchSurface(PointCloud<PointXYZ>::Ptr outcloud){
    // Constantes
    const float min_yaw = -Util::PI/2.0; //-90°
    const float max_yaw = Util::PI/2.0;  // 90°
    const float yaw_step = Util::PI/4.0; // 45°
    float yaw = min_yaw;
    ROS_INFO("Se inicia búsqueda de superficie");
    // Mirar al frente
    r_driver->head->lookAt(Util::BASE_FRAME, 2, 0, 0);
    // Contenedor de nube de puntos
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>()),
                              cloud_subsampled(new PointCloud<PointXYZ>()),
                              cloud_surface(new PointCloud<PointXYZ>());
    // Iterar y mirar alrededor
    while (yaw <= max_yaw){
        // Rotar cabeza
        r_driver->head->rotate(Util::BASE_FRAME, yaw);
        // Obtener nube de puntos desde kinect
        ROS_DEBUG("place: Esperando nube de puntos...");
        cloud = r_driver->sensors->kinect->getNewCloud();
        ROS_DEBUG("place: Nube recibida");
        // Submuestrear nube
        cloud_subsampled = Util::subsampleCloud(cloud, Util::SUBSAMPLE_LEAFSIZE);
        // Buscar un plano adecuado
        
        if (Util::searchPlacingSurface(cloud_subsampled, cloud_surface, 0.4, 0.8, Util::DEFAULT_DESIRED_PITCH)){
            outcloud = cloud_surface;
            return true;
        }
        ROS_INFO("place: No fue posible encontrar una superficie de placing");
        yaw += yaw_step;
    }
    // Si no se encontró, buscar hacia el otro lado
    // Subir la mirada e iterar
    while (yaw >= min_yaw){
        // Robar cabeza
        r_driver->head->rotate(Util::BASE_FRAME, yaw);
        // Obtener nube de puntos de kinect

        // Buscar plano adecuado

        yaw -= yaw_step;
    }
    ROS_INFO("Búsqueda de superficie finalizada");
    return true;
}




/**
 * main: Ejecuta todo lo necesario para efectuar placing
 * @param argc: Cuenta de argumentos
 * @param argv: Recibe como argumento la mano donde está el objeto
 * @return: código de error
 */
int main(int argc, char **argv){
    // capturar argumento
    if (argc < 2){
        ROS_ERROR("Debe ingresar brazo que posee objeto [l|r]\n");
        exit(1);
    }
    grasp_arm = argv[1][0];
    if (grasp_arm != 'l' and grasp_arm != 'r'){
        ROS_ERROR("Error: Debe ingresar brazo válido [l|r]");
        exit(1);
    }
    ROS_INFO("Asumiendo objeto en gripper %s", grasp_arm == 'l' ? "izquierdo" : "derecho");
    // Iniciar nodo ROS
    ros::init(argc, argv, "placing_node");
    ros::NodeHandle nh;
    // Verbosidad para debugging
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }
    // Para terminar con ctrl+c
    signal(SIGINT, signalHandler);
    // Iniciando robot driver
    ROS_DEBUG("PLACE: Iniciando RobotDriver");
    r_driver = new RobotDriver();
    ROS_DEBUG("PLACE: RobotDriver Creado e iniciado");
    
    // 1) BUSCAR SUPERFICIE
/*    PointCloud<PointXYZ>::Ptr surface_cloud (new PointCloud<PointXYZ>());
    if (not searchSurface(surface_cloud)){
        ROS_ERROR("No se pudo obtener superficie");
        exit(1);
    }*/

    //   ZONA DE PRUEBAS
/*    // Mover 1m hacia atrás y mirar al frente
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = Util::BASE_FRAME;
    pose.pose.position.x = 2;
    pose.pose.position.y = pose.pose.position.z = 0;
    pose.pose.orientation = Util::coefsToQuaternionMsg(1,1,0);
    r_driver->base->goToPose(pose);*/

/*    r_driver->lgripper->setOpening(1, 400);
    r_driver->rgripper->setOpening(1, 400);
    r_driver->lgripper->setOpening(0, 400);
    r_driver->rgripper->setOpening(0, 400);*/
    ros::Publisher cloud_pub = nh.advertise<PointCloud<PointXYZ> >("nube_maravillosa", 1);
    PointCloud<PointXYZ>::Ptr kinect_cloud (new PointCloud<PointXYZ>());
    ROS_DEBUG("PLACE: creado topico e iniciando ciclo");

    while (1){
        kinect_cloud = r_driver->sensors->kinect->getNewCloud();
        ROS_DEBUG("PLACE: N° Puntos: %d", (int)kinect_cloud->points.size());
        cloud_pub.publish(*kinect_cloud);
        ros::Duration(0.4).sleep();    
    }


    // END ZONA DE PRUEBAS





    endProgram(0);
}


/*

    TODO:
        - Revisar problema de head driver de no hacer nada a veces (lanzar timeout)

 */