/* 
Este algoritmo se basa en el flujo explicitado en "~/Escritorio/Memoria/Conceptos"
 
Estado inicial:
    El PR2 se encuentra parado cerca de la superficie
    El PR2 posee objeto en el gripper. Se sabe qué griper lo tiene.
    Se tiene guardada la nube de puntos que la representa
    Se tiene guardada la pose escogida para dejar el objeto
    El PR2 está mirando al punto donde se quiere posicionar objeto

Pseudocódigo:
    
    (1)- Planear trayectoria (se necesita pose de la etapa anterior)
    - if factible:
        - moverse
        - if problemas:
            - ABORTAR
        - else RETURN
    - else:
        - Elegir otro punto en superficie (Usar paso anterior?)
        - if no quedan puntos:
            - ABORTAR
        - else goto 1


Componentes necesarias:
LIBRERÍAS
    ROS
        ros/ros : para todo ros
        Mensajes:
            geometry_msgs/PoseStamped : para la pose final del objeto
            sensor_msgs/PointCloud2 : Nube de puntos con la superficie encontrada para placing
            moveit_msgs/DisplayTrajectory : Para visualizar en Rviz trayectorias planeadas
        Servicios:
            memoria/SearchSurface : Servicio para búsqueda de superficie.

    PCL para procesamiento de nube de puntos del Kinect
    MOVEIT
        moveit/planning_interface/planning_interface 
        moveit/move_group_interface/move_group.h
    OTROS
        string
        csignal : Para salir con ctrl+C

TODO:
    - Implementar servicio (o método) getPlacingPose
    - Implementar placeObject
    - Actualizar diagramas de flujo con arquitectura anterior
    - Cambiar diagrama de flujo en parte de Tuck Arms. Debe tuckearse sólo el brazo que no tiene el objeto. Esto debe saberse de antemano.
    - Terminar moveToSurface mirando centroide de superficie
    - Considerar altura del torso al llegar a la superficie
    - Considerar cambiar servicios por métodos en una clase auxiliar
*/
#include <vector>
#include <csignal>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <memoria/SearchSurface.h>
#include <memoria/GoToPose.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_listener.h>

using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
// CONSTANTES
const string SEARCH_SURFACE_SRV_NAME = "search_surface";
const string GO_TO_POSE_SRV_NAME = "go_to_pose";
const string ROBOT_FRAME = "base_footprint";
const float ROBOT_FRONT_MARGIN = 0.5; // delta de distancia para no chocar
const float WAIT_TF_TIMEOUT = 1.0;
// Dummies, para recrear condiciones iniciales
geometry_msgs::PoseStamped final_object_pose;
const string ACTIVE_ARM = "right_arm";
// Variables auxiliares
ros::Publisher aux_pointcloud_pub;
ros::Publisher aux_posestamped_pub;
ros::Publisher aux_posestamped2_pub;
ros::Publisher aux_point_pub;
// VARIABLES GLOBALES
sensor_msgs::PointCloud2 surface;
ros::ServiceClient search_surface_client;
ros::ServiceClient go_to_pose_client;
memoria::SearchSurface search_surface_srv;
memoria::GoToPose go_to_pose_srv;

// MÉTODOS
void endProgram(int retcode){
    ros::shutdown();
    exit(retcode);
}
void signalHandler( int signum ){
    endProgram(EXIT_SUCCESS);
}
void searchSurface(){
    /*
    Llama al servicio memoria/SearchSurface y espera que termine.
    Resultado de retorno:
        - PointCloud2 con superficie, en frame_id del kinect
        - memoria/ErrorMsg con código de error.
    */
    if (search_surface_client.call(search_surface_srv)){
        // Verificar información obtenida   
        if (search_surface_srv.response.error.retcode != 0){
            ROS_ERROR("Error '%d' en búsqueda de superficie: '%s'",search_surface_srv.response.error.retcode, search_surface_srv.response.error.what.c_str());
            endProgram(1);
        }
        ROS_INFO("Almacenando superficie encontrada");
        surface = search_surface_srv.response.pointcloud;
        //ROS_INFO("nube: Tamaño=%d",surface)
        aux_pointcloud_pub.publish(surface);
    }
    else{
        ROS_ERROR("Error al llamar al servicio %s",SEARCH_SURFACE_SRV_NAME.c_str());
        endProgram(1);
    }
}
void moveToSurface(){
    // ****** Elegir punto cerca de la superficie
    geometry_msgs::PoseStamped pose_goal;
    // Convertir nube de ROS a PCL
    PointCloud pcl_surface;
    pcl::fromROSMsg(surface, pcl_surface);
    // Crear elemento KdTree
    pcl::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
    kdtree->setInputCloud(pcl_surface.makeShared()); // NO ELIMINAR EL MAKESHARED POR NINGUN MOTIVO
    vector<int> nearest_index(1);
    vector<float> nearest_dist(1);
    ROS_INFO("Buscando punto más cercano...");
    kdtree->nearestKSearch(pcl::PointXYZ(0,0,0), 1, nearest_index, nearest_dist);
    ROS_INFO("Punto más cercano es (%f, %f, %f) a distancia %f", pcl_surface.points[nearest_index[0]].x,pcl_surface.points[nearest_index[0]].y, pcl_surface.points[nearest_index[0]].z, nearest_dist[0]);
    // publicar punto más cercano
    geometry_msgs::PointStamped closest_point_kinect;
    closest_point_kinect.header.frame_id =  "head_mount_kinect_ir_optical_frame";
    closest_point_kinect.point.x = pcl_surface.points[nearest_index[0]].x;
    closest_point_kinect.point.y = pcl_surface.points[nearest_index[0]].y;
    closest_point_kinect.point.z = pcl_surface.points[nearest_index[0]].z;
    aux_point_pub.publish(closest_point_kinect);
    // ****** Ir a la superficie
    // Transformar punto de pose más cercana desde kinect frame hasta robot frame
    tf::TransformListener tf_listener;
    tf::StampedTransform stamped_tf;
    try{
        ROS_INFO("Esperando transformación disponible...");
        while (not tf_listener.waitForTransform(closest_point_kinect.header.frame_id, ROBOT_FRAME, ros::Time(0), ros::Duration(WAIT_TF_TIMEOUT))){
            ROS_ERROR("Transformación no pudo ser obtenida en %f segundos",WAIT_TF_TIMEOUT);
            //iterar
        }
        ROS_INFO("Transformación obtenida");
        tf_listener.lookupTransform(closest_point_kinect.header.frame_id, ROBOT_FRAME, ros::Time(0), stamped_tf);
    }
    catch(tf::TransformException ex){
        ROS_ERROR("Excepción al obtener transformación: '%s'", ex.what());
    }
    geometry_msgs::PointStamped closest_point;
    tf_listener.transformPoint(ROBOT_FRAME, closest_point_kinect, closest_point);
    // Obtener pose_goal con pose cercana - delta
    tf::Vector3 closest_point_floor_vector (closest_point.point.x,closest_point.point.y,0);
    double closest_point_floor_distance = closest_point_floor_vector.length();
    ROS_INFO("Distancia punto más cercano: %f (%f)",closest_point_floor_distance,nearest_dist[0]);
    closest_point_floor_vector.normalize();
    closest_point_floor_vector*=closest_point_floor_distance-ROBOT_FRONT_MARGIN;
    ROS_INFO("Distancia pose factible (resté %f): %f",ROBOT_FRONT_MARGIN,closest_point_floor_vector.length());
    geometry_msgs::PoseStamped closest_pose;
    closest_pose.header.frame_id = closest_point.header.frame_id;
    closest_pose.pose.position.x = closest_point_floor_vector.x();
    closest_pose.pose.position.y = closest_point_floor_vector.y();
    closest_pose.pose.position.z = closest_point_floor_vector.z();
    tf::Vector3 start_angle (1,0,0), zaxis (0,0,1);
    double rotation_angle = (closest_point_floor_vector.y() < 0 ? -1 : 1)* start_angle.angle(closest_point_floor_vector);
    ROS_INFO("Angulo rotación: %f",rotation_angle);
    tf::Quaternion tf_orientation (zaxis,rotation_angle);
    tf::quaternionTFToMsg(tf_orientation, closest_pose.pose.orientation);
    aux_posestamped2_pub.publish(closest_pose);
    // Mover 
    go_to_pose_srv.request.pose = closest_pose;
    if (go_to_pose_client.call(go_to_pose_srv)){
        // Verificar qué se obtuvo
        if (go_to_pose_srv.response.error.retcode != 0){
            ROS_ERROR("Error '%d' en ir a la pose: '%s'",go_to_pose_srv.response.error.retcode, go_to_pose_srv.response.error.what.c_str());
            endProgram(1);
        }
        ROS_INFO("PR2 se ha movido con éxito a la pose solicitada");
        aux_posestamped_pub.publish(pose_goal);
    }
    else{
        ROS_ERROR("Error al llamar al servicio %s",GO_TO_POSE_SRV_NAME.c_str());
        endProgram(1);
    }
}
void getPlacingPose(){

}
void placeObject(geometry_msgs::PoseStamped pose_goal){
    /* Función encargada de posicionar objeto en cierta mano, en cierta pose.
    Estrictamente, lo que esta función hace es:
        - Planea trayectoria de cierto gripper (conocido) a pose de placing obtenida
        - Mueve gripper a esa pose
        - Suelta el objeto
        - Retira gripper sin botar el objeto (Quizás moviéndose en misma dirección en que apunta el gripper en pose de placing)

    Recibe:
        - Pose deseada
        - grupo*/
    // Creando pose dummy
    

    // Planear 
}
int main(int argc, char **argv){
    // Inicializar nodo ROS
    ros::init(argc, argv, "moveit_tutoriales");
    // Directo de los tutoriales
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ROS_INFO("Comenzando ejecución");
    ros::NodeHandle nh;
    
    // Conectar a los servicios
    ROS_INFO("Creando cliente para '%s'",SEARCH_SURFACE_SRV_NAME.c_str());
    search_surface_client = nh.serviceClient<memoria::SearchSurface>(SEARCH_SURFACE_SRV_NAME);
    ROS_INFO("Creando cliente para '%s'",GO_TO_POSE_SRV_NAME.c_str());
    go_to_pose_client = nh.serviceClient<memoria::GoToPose>(GO_TO_POSE_SRV_NAME);

    // Publicar dummies
    aux_pointcloud_pub = nh.advertise<PointCloud> ("plano_dummy",1);
    aux_posestamped_pub = nh.advertise<geometry_msgs::PoseStamped> ("pose_dummy",1);
    aux_posestamped2_pub = nh.advertise<geometry_msgs::PoseStamped> ("pose_mas_cercana",1);
    aux_point_pub = nh.advertise<geometry_msgs::PointStamped> ("punto_mas_cercano",1);
    
    signal(SIGINT, signalHandler);
    ROS_INFO("Inicializando %s",ACTIVE_ARM.c_str());
    moveit::planning_interface::MoveGroup active_arm(ACTIVE_ARM);
    ROS_INFO("Frame del planning: %s; end effector: %s",active_arm.getPlanningFrame().c_str(), active_arm.getEndEffectorLink().c_str());
    moveit::planning_interface::MoveGroup::Plan plan;
    // Comienza ejecución del megaprograma
    ROS_INFO("Iniciando búsqueda de superficie");
    searchSurface();
    ROS_INFO("Iniciando desplazamiento hacia superficie");
    moveToSurface();
    /*ROS_INFO("Iniciando cálculo de pose para objeto");
    getPlacingPose();
    ROS_INFO("Iniciando placing");
    placeObject();
    // ROS_INFO("Placing finalizado con éxito.");*/

    return 0;
}