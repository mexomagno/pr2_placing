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
    - Definir bien la arquitectura de la memoria. Es éste el nodo principal?
        Puede pensarse como que es este el nodo principal, y los demás son simples servicios. 
        Osea, primero se obtiene superficie con servicio buscar_superficie, que retorna el pointcloud.
        Luego, se mueve el robot con el servicio para movimiento, según ubicación de la superficie
        A continuación, se obtiene pose ideal de placing usando servicio destinado a esto. 
        Quizás el mismo servicio anterior hace al robot mirar a este punto, o podemos hacerlo dentro de este archivo.
        En este punto se tiene superficie, se está situado al lado, y se tiene pose deseada para el objeto y se la está mirando.
        Con lo anterior tenemos bien recreadas las condiciones iniciales necesarias para proceder al placing.
        - Actualizar diagramas de flujo con arquitectura anterior
    - Cambiar diagrama de flujo en parte de Tuck Arms. Debe tuckearse sólo el brazo que no tiene el objeto. Esto debe saberse de antemano.
*/
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <memoria/SearchSurface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <csignal>

using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
// CONSTANTES
const string SEARCH_SURFACE_SRV_NAME = "search_surface";
// Dummies, para recrear condiciones iniciales
sensor_msgs::PointCloud2 surface;
geometry_msgs::PoseStamped final_pose;
const string ACTIVE_ARM = "right_arm";
// Variables auxiliares
ros::Publisher aux_pointcloud_pub;
// VARIABLES GLOBALES
ros::ServiceClient search_surface_client;
memoria::SearchSurface search_surface_srv;

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
    Resultado de retorno es un pointcloud y un memoria/ErrorMsg.
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

}
void getPlacingPose(){

}
void placeObject(){

}
int main(int argc, char **argv){
    // Inicializar nodo ROS
    ros::init(argc, argv, "moveit_tutoriales");
    // Directo de los tutoriales
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ROS_INFO("Comenzando ejecución");
    ros::NodeHandle nh;
    ROS_INFO("Creando cliente para 'search_surface'");
    search_surface_client = nh.serviceClient<memoria::SearchSurface>(SEARCH_SURFACE_SRV_NAME);
    aux_pointcloud_pub = nh.advertise<PointCloud> ("plano_dummy",1);
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
    ROS_INFO("Iniciando cálculo de pose para objeto");
    getPlacingPose();
    ROS_INFO("Iniciando placing");
    placeObject();
    ROS_INFO("Placing finalizado con éxito.");

    return 0;
}