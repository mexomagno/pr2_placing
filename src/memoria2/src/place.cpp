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
#include "RobotDriver/RobotDriver.h"
#include "Util/Util.h"


using namespace std;
using namespace pcl;

// CONSTANTES

// VARIABLES GLOBALES
RobotDriver *r_driver;
char grasp_arm;

// Auxiliares y cosas para visualizar
ros::Publisher cloud_pub;
ros::Publisher cloud_pub2;
ros::Publisher point_pub;
ros::Publisher point_pub2;
ros::Publisher pose_pub;

// Pre-declaración de métodos
void endProgram(int retcode);
void signalHandler(int signum);
bool searchSurface(PointCloud<PointXYZ>::Ptr &cloud_out);
bool moveToSurface(PointCloud<PointXYZ>::Ptr cloud);
bool getPlacingPose(geometry_msgs::PoseStamped &pose_out);
bool scanGripper(PointCloud<PointXYZ>::Ptr &object_out, PointCloud<PointXYZ>::Ptr &gripper_out);

// MÉTODOS

void endProgram(int retcode){
    ROS_INFO("Terminando programa...");
    delete r_driver;
    ros::shutdown();
    printf("Programa completamente finalizado :') (código %d)\n", retcode);
    exit(retcode);
}
void signalHandler(int signum){
    ROS_INFO("Se recibe Ctrl+C");
    endProgram(0);
}
bool searchSurface(PointCloud<PointXYZ>::Ptr &cloud_out){
    // Constantes
    const float min_yaw = -Util::PI/2.0; //-90°
    const float max_yaw = Util::PI/2.0;  // 90°
    const float yaw_step = Util::PI/4.0; // 45°
    float yaw = min_yaw;
    ROS_INFO("Se inicia búsqueda de superficie");
    // Mirar al frente
    r_driver->head->lookAt(Util::BASE_FRAME, 1.5, 0, 0);
    // Contenedor de nube de puntos
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>()),
                              cloud_subsampled(new PointCloud<PointXYZ>()),
                              cloud_surface(new PointCloud<PointXYZ>());
    // Iterar y mirar alrededor
    while (yaw <= max_yaw){
        ROS_DEBUG("PLACE: Rotando a yaw = %f", Util::toGrad(yaw));
        // Rotar cabeza
        r_driver->head->rotate(Util::BASE_FRAME, yaw);
        ros::Duration(Util::KINECT_STABILIZE_TIME).sleep();
        // Obtener nube de puntos desde kinect
        ROS_DEBUG("PLACE: Esperando nube de puntos...");
        cloud = r_driver->sensors->kinect->getNewCloud();
        ROS_DEBUG("PLACE: Nube de %d puntos recibida", (int)cloud->points.size());
        // Submuestrear nube
        ROS_DEBUG("PLACE: Submuestreando...");
        cloud_subsampled = Util::subsampleCloud(cloud, Util::SUBSAMPLE_LEAFSIZE);
        ROS_DEBUG("PLACE: %d puntos tras submuestreo", (int)cloud_subsampled->points.size());
        // Buscar un plano adecuado
        ROS_DEBUG("PLACE: Buscando superficie para placing...");
        if (Util::searchPlacingSurface(cloud_subsampled, cloud_surface, 0.3, 0.8, Util::DEFAULT_DESIRED_PITCH)){
            ROS_DEBUG("PLACE: Encontrada");
            cloud_out = cloud_surface;
            return true;
        }
        ROS_INFO("PLACE: No encontrada. Iterando...");
        yaw += yaw_step;
    }
    // Si no se encontró, buscar hacia el otro lado
    // Subir la mirada e iterar
    r_driver->head->lookAt(Util::BASE_FRAME, 0, 2, 1.0);
    while (yaw >= min_yaw){
        ROS_DEBUG("PLACE: Rotando a yaw = %f", Util::toGrad(yaw));
        // Rotar cabeza
        r_driver->head->rotate(Util::BASE_FRAME, yaw);
        ros::Duration(Util::KINECT_STABILIZE_TIME).sleep();
        // Obtener nube de puntos desde kinect
        ROS_DEBUG("PLACE: Esperando nube de puntos...");
        cloud = r_driver->sensors->kinect->getNewCloud();
        ROS_DEBUG("PLACE: Nube de %d puntos recibida", (int)cloud->points.size());
        // Submuestrear nube
        ROS_DEBUG("PLACE: Submuestreando...");
        cloud_subsampled = Util::subsampleCloud(cloud, Util::SUBSAMPLE_LEAFSIZE);
        ROS_DEBUG("PLACE: %d puntos tras submuestreo", (int)cloud_subsampled->points.size());
        // Buscar un plano adecuado
        ROS_DEBUG("PLACE: Buscando superficie para placing...");
        if (Util::searchPlacingSurface(cloud_subsampled, cloud_surface, 0.3, 0.8, Util::DEFAULT_DESIRED_PITCH)){
            ROS_DEBUG("PLACE: Encontrada");
            cloud_out = cloud_surface;
            return true;
        }
        ROS_INFO("PLACE: No encontrada. Iterando...");
        yaw -= yaw_step;
    }
    return false;
}
bool moveToSurface(PointCloud<PointXYZ>::Ptr cloud){
    // Transformar la nube de Odom a Base;
    Eigen::Matrix4f transformation = Util::getTransformation(cloud->header.frame_id, Util::BASE_FRAME);
    PointCloud<PointXYZ>::Ptr cloud_base (new PointCloud<PointXYZ>());
    transformPointCloud(*cloud, *cloud_base, transformation);
    cloud_base->header.frame_id = Util::BASE_FRAME;
    // Elegir punto más cercano a la superficie (respecto a la base!)
    geometry_msgs::PointStamped closest_point;
    float closest_point_distance;
    Util::getClosestPoint(cloud_base, closest_point, closest_point_distance);
    point_pub.publish(closest_point);
    // Obtener una pose adecuada para llegar a ese punto
    // la pose se encuentra un poquito más atrás que el punto.
    ROS_DEBUG("PLACE: Punto más cercano está a %fm", closest_point_distance);
    if (closest_point_distance > Util::ROBOT_FRONT_MARGIN){
        ROS_DEBUG("PLACE: Debo acercarme");
        tf::Vector3 closest_point_floor_vector (closest_point.point.x, closest_point.point.y, 0);
        float closest_point_floor_distance = closest_point_floor_vector.length();
        closest_point_floor_vector.normalize();
        closest_point_floor_vector*=closest_point_floor_distance - Util::ROBOT_FRONT_MARGIN;
        ROS_DEBUG("PLACE: Me ubicaré a %fm de la pose actual", closest_point_floor_vector.length());
        geometry_msgs::PoseStamped closest_pose;
        closest_pose.header.frame_id = Util::BASE_FRAME;
        closest_pose.pose.position.x = closest_point_floor_vector.x();
        closest_pose.pose.position.y = closest_point_floor_vector.y();
        closest_pose.pose.position.z = closest_point_floor_vector.z();
        closest_pose.pose.orientation = Util::coefsToQuaternionMsg(closest_point_floor_vector.x(), closest_point_floor_vector.y(), 0);
        // Ver pose
        pose_pub.publish(closest_pose);
        ROS_DEBUG("PLACE: Yendo a la pose...");
        if (not r_driver->base->goToPose(closest_pose)){
            ROS_ERROR("PLACE: No se pudo ir a la pose");
        }
        ROS_DEBUG("PLACE: Llegué a la pose");
    }
    ROS_DEBUG("Mirando al centroide de la superficie");
    // Nos movimos: actualizar entonces la transformación entre odom y base
    transformation = Util::getTransformation(cloud->header.frame_id, Util::BASE_FRAME);
    // Transformar superficie
    transformPointCloud(*cloud, *cloud_base, transformation);
    // Obtener centroide (respecto a la base)
    geometry_msgs::Point centroid = Util::getCloudCentroid(cloud_base);
    // Mirar hacia el centroide
    r_driver->head->lookAt(Util::BASE_FRAME, centroid.x, centroid.y, centroid.z);    
}
bool getPlacingPose(geometry_msgs::PoseStamped &pose_out){
    // Scannear gripper
    PointCloud<PointXYZ>::Ptr gripper_scan (new PointCloud<PointXYZ>()),
                              object_scan (new PointCloud<PointXYZ>());
    return scanGripper(object_scan, gripper_scan);
    // Filtrar gripper del scan
    // Obtener mejor superficie
}
bool scanGripper(PointCloud<PointXYZ>::Ptr &object_out, PointCloud<PointXYZ>::Ptr &gripper_out){
    // Mover gripper a posición inicial de scanning
    geometry_msgs::PoseStamped scan_pose;
    scan_pose.header.frame_id = Util::BASE_FRAME;
    scan_pose.pose.position.x = Util::scan_position[0];
    scan_pose.pose.position.y = Util::scan_position[1];
    scan_pose.pose.position.z = Util::scan_position[2];
    // Copia no constante de la orientación inicial de scanning
    float scan_orientation[3];
    scan_orientation[0] = Util::scan_orientation[0];
    scan_orientation[1] = Util::scan_orientation[1];
    scan_orientation[2] = Util::scan_orientation[2];
    scan_orientation[0] += Util::SCAN_ROLL_DELTA;
    scan_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(scan_orientation[0], scan_orientation[1], scan_orientation[2]);
    string GRIPPER_FRAME = (grasp_arm == 'l' ? "l" : "r") + Util::GRIPPER_FRAME_SUFFIX;
    if (grasp_arm == 'l')
        r_driver->lgripper->goToPose(scan_pose);
    else
        r_driver->rgripper->goToPose(scan_pose);
    ros::Duration(Util::GRIPPER_STABILIZE_TIME).sleep();
    // Contenedores para objetos creados en el ciclo
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>()), 
                              cloud_near(new PointCloud<PointXYZ>()),
                              cloud_subsampled(new PointCloud<PointXYZ>()),
                              cloud_base(new PointCloud<PointXYZ>());
    vector<PointCloud<PointXYZ> > cloud_scans;
    Eigen::Matrix4f transformation;
    // Scanear
    while ( 1 ){
        // Pedir nube de kinect
        cloud = r_driver->sensors->kinect->getNewCloud();
        // obtener transformación de kinect a wrist
        transformation = Util::getTransformation(Util::KINECT_FRAME, GRIPPER_FRAME);
        // Eliminar puntos lejanos
        PassThrough<PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.6 - Util::SCAN_PASSTHROUGH_Z/2.0, 0.6 + Util::SCAN_PASSTHROUGH_Z/2.0); // HARDCODEADO. FORMA CORRECTA ES CON TF?
        pass.filter(*cloud_near);
        // submuestrear
        cloud_subsampled = Util::subsampleCloud(cloud_near, Util::SCAN_LEAFSIZE);
        // Transformar y agregar a scans
        transformPointCloud(*cloud_subsampled, *cloud_base, transformation);
        cloud_scans.push_back(*cloud_base);
        // Si ya escaneamos todas las perspectivas, juntarlas y retornar
        scan_orientation[0] += Util::SCAN_ROLL_DELTA;
        if (scan_orientation[0] > 2*Util::PI){
            PointCloud<PointXYZ>::Ptr merged(new PointCloud<PointXYZ>());
            for (int i=0; i<cloud_scans.size(); i++){
                *merged += cloud_scans[i];
            }
            // Filtrar gripper
            Util::gripperFilter(merged, object_out, gripper_out);
            return true;
        }
        ROS_DEBUG("UTIL: Posicionándose psra siguiente scan");
        scan_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(scan_orientation[0], scan_orientation[1], scan_orientation[2]);
        if (grasp_arm == 'l')
            r_driver->lgripper->goToPose(scan_pose);
        else
            r_driver->rgripper->goToPose(scan_pose);
        ros::Duration(Util::GRIPPER_STABILIZE_TIME).sleep();
    }
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

/*    ros::Publisher cloud_pub = nh.advertise<PointCloud<PointXYZ> >("nube_maravillosa", 1);
    PointCloud<PointXYZ>::Ptr kinect_cloud (new PointCloud<PointXYZ>());
    ROS_DEBUG("PLACE: creado topico e iniciando ciclo");
    while (1){
        kinect_cloud = r_driver->sensors->kinect->getNewCloud();
        ROS_DEBUG("PLACE: N° Puntos: %d", (int)kinect_cloud->points.size());
        cloud_pub.publish(*kinect_cloud);
        ros::Duration(0.4).sleep();    
    }*/

/*    geometry_msgs::PoseStamped placing_pose;
    if (not getPlacingPose(placing_pose)){
        ROS_ERROR("PLACE: No se pudo obtener una pose de placing");
        endProgram(1);
    }*/

    ROS_DEBUG("PLACE: Llevando gripper izquierdo al frente");
    geometry_msgs::PoseStamped gripper_pose;
    gripper_pose.header.frame_id = Util::BASE_FRAME;
    gripper_pose.pose.position.x = 0.7;
    gripper_pose.pose.position.y = 0.3;
    gripper_pose.pose.position.z = 1.2;
    r_driver->lgripper->goToPose(gripper_pose);
    ROS_DEBUG("PLACE: Llevando gripper derecho al frente");
    gripper_pose.pose.position.y = -0.3;
    r_driver->rgripper->goToPose(gripper_pose);
    ros::Duration(1).sleep();
    ROS_INFO("PLACE: FIN");

    // END ZONA DE PRUEBAS


/*
    cloud_pub = nh.advertise<PointCloud<PointXYZ> >("superficie", 1);
    cloud_pub2 = nh.advertise<PointCloud<PointXYZ> >("punto_trans_pc", 1);
    point_pub = nh.advertise<geometry_msgs::PointStamped>("punto_mas_cercano", 1);
    point_pub2 = nh.advertise<geometry_msgs::PointStamped>("punto_trans", 1);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose_mas_cercana", 1);
    ros::Duration(1).sleep();
    
    PointCloud<PointXYZ>::Ptr surface_cloud (new PointCloud<PointXYZ>());
    if (not searchSurface(surface_cloud)){
        ROS_ERROR("No se pudo obtener superficie");
        exit(1);
    }
    ROS_INFO("PLACE: Superficie encontrada (frame: %s). Publicando...", surface_cloud->header.frame_id.c_str());
    cloud_pub.publish(*surface_cloud);
    ROS_INFO("PLACE: Dirigiéndose hacia ella");
    moveToSurface(surface_cloud);
    ros::Duration(1).sleep();
    
    ROS_INFO("PLACE: FIN");*/


    endProgram(0);
}


/*

    TODO:
        - Revisar problema de head driver de no hacer nada a veces (lanzar timeout)
    
    opcionales:
        - Evaluar corregir vista de la superficie encontrada (una vez encontrada, mirar hacia ella, volver a buscar y corregir nube)
        - Evaluar reparar head driver para rotar pitch
        - Evaluar eliminar pose inicial del head driver al inicializarlo
 */