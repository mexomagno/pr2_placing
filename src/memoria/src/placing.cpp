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
*/
#include <vector>
#include <csignal>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <memoria/SearchSurface.h>
#include <memoria/GoToPose.h>
#include <memoria/LookAt.h>
#include <memoria/LookAtMsg.h>
#include <pcl/common/centroid.h>
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
using namespace pcl;

// typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
// CONSTANTES
const string SEARCH_SURFACE_SRV_NAME = "search_surface";
const string GO_TO_POSE_SRV_NAME = "go_to_pose";
const string LOOKAT_SRV_NAME = "look_at";
const string BASE_FRAME = "base_footprint";
const string KINECT_FRAME = "head_mount_kinect_ir_optical_frame";
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
ros::NodeHandle *global_nh;
// MÉTODOS
void endProgram(int retcode){
    ros::shutdown();
    exit(retcode);
}
void signalHandler( int signum ){
    endProgram(EXIT_SUCCESS);
}
float toGrad(float rad){
    return rad*180.0/3.1415;
}
float toRad(int grados){
    return grados*3.1415/180.0;
}
geometry_msgs::Quaternion coefsToQuaternionMsg(float a, float b, float c){
    /* 
    Recibe: a, b, c, coeficientes de un plano
    Retorna: Quaternion con la orientación de la normal del plano.
    */
    float vmod=sqrt(a*a+b*b+c*c);
    float pitch = -(1.0*asin(1.0*c/vmod));
    float yaw = (a==0?toRad(90):atan(b/a)) + (a>=0?0:toRad(180));
    return tf::createQuaternionMsgFromRollPitchYaw(0,pitch,yaw);
}
 
void searchSurface(PointCloud<PointXYZ>::Ptr &surface_pc){
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
        // Guardar superficie como nube PCL
        PCLPointCloud2 surface_pc2;
        pcl_conversions::toPCL(surface, surface_pc2);
        fromPCLPointCloud2(surface_pc2, *surface_pc);
        //ROS_INFO("nube: Tamaño=%d",surface)
        // aux_pointcloud_pub.publish(surface);
        aux_pointcloud_pub.publish(*surface_pc);
    }
    else{
        ROS_ERROR("Error al llamar al servicio %s",SEARCH_SURFACE_SRV_NAME.c_str());
        endProgram(1);
    }
}
void moveToSurface(PointCloud<PointXYZ>::Ptr surface_pc){
    // ****** Elegir punto cerca de la superficie
    geometry_msgs::PoseStamped pose_goal;
    // Crear elemento KdTree y buscar punto más cercano
    pcl::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
    kdtree->setInputCloud(surface_pc->makeShared()); // NO ELIMINAR EL MAKESHARED POR NINGUN MOTIVO
    vector<int> nearest_index(1);
    vector<float> nearest_dist(1);
    ROS_INFO("Buscando punto más cercano (a la kinect)...");
    kdtree->nearestKSearch(pcl::PointXYZ(0,0,0), 1, nearest_index, nearest_dist);
    ROS_INFO("Punto más cercano es (%f, %f, %f) a distancia %f", surface_pc->points[nearest_index[0]].x,surface_pc->points[nearest_index[0]].y, surface_pc->points[nearest_index[0]].z, nearest_dist[0]);
    // convertir puntode PCL a ROSMSG
    geometry_msgs::PointStamped closest_point_kinect;
    closest_point_kinect.header.frame_id =  KINECT_FRAME;
    closest_point_kinect.point.x = surface_pc->points[nearest_index[0]].x;
    closest_point_kinect.point.y = surface_pc->points[nearest_index[0]].y;
    closest_point_kinect.point.z = surface_pc->points[nearest_index[0]].z;
    // ****** Ir a la superficie
    // Transformar punto más cercano de kinect_frame a base_footprint
    tf::TransformListener tf_listener;
    tf::StampedTransform stamped_tf;
    try{
        ROS_INFO("Esperando transformación disponible...");
        while (not tf_listener.waitForTransform(KINECT_FRAME, BASE_FRAME, ros::Time(0), ros::Duration(WAIT_TF_TIMEOUT))){
            ROS_ERROR("Transformación no pudo ser obtenida en %f segundos",WAIT_TF_TIMEOUT);
            //iterar
        }
        ROS_INFO("Transformación obtenida");
        tf_listener.lookupTransform(KINECT_FRAME, BASE_FRAME, ros::Time(0), stamped_tf);
    }
    catch(tf::TransformException ex){
        ROS_ERROR("Excepción al obtener transformación: '%s'", ex.what());
    }
    geometry_msgs::PointStamped closest_point;
    tf_listener.transformPoint(BASE_FRAME, closest_point_kinect, closest_point);
    // Publicar para visualización.
    aux_point_pub.publish(closest_point);
    // Obtener pose_goal = punto_mas_cercano - delta (vectorial)
    tf::Vector3 closest_point_floor_vector (closest_point.point.x,closest_point.point.y,0);
    double closest_point_floor_distance = closest_point_floor_vector.length();
    ROS_INFO("Distancia punto más cercano: %f",closest_point_floor_distance);
    // Obtengo vector unitario apuntando a punto más cercano
    closest_point_floor_vector.normalize();
    // Multiplico vector por distancia - delta (obtengo vector a posición final)
    closest_point_floor_vector*=closest_point_floor_distance-ROBOT_FRONT_MARGIN;
    ROS_INFO("Distancia pose factible (reste %f): %f",ROBOT_FRONT_MARGIN,closest_point_floor_vector.length());
    geometry_msgs::PoseStamped closest_pose;
    closest_pose.header.frame_id = BASE_FRAME;
    closest_pose.pose.position.x = closest_point_floor_vector.x();
    closest_pose.pose.position.y = closest_point_floor_vector.y();
    closest_pose.pose.position.z = closest_point_floor_vector.z();
    // Obtener orientación del vector
    // tf::Vector3 start_angle (1,0,0), zaxis (0,0,1);
    // double rotation_angle = (closest_point_floor_vector.y() < 0 ? -1 : 1)* start_angle.angle(closest_point_floor_vector);
    // ROS_INFO("Angulo rotación: %f",rotation_angle);
    /*    tf::Vector3 zaxis(0,0,1);
    double rotation_angle = -acos(closest_point_floor_vector.x());
    ROS_INFO("Angulo rotacion: %f", rotation_angle);
    tf::Quaternion tf_orientation (zaxis,rotation_angle);
    tf::quaternionTFToMsg(tf_orientation, closest_pose.pose.orientation);*/
    closest_pose.pose.orientation = coefsToQuaternionMsg(closest_point_floor_vector.x(), closest_point_floor_vector.y(), 0);
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
    // Mirar centroide de la mesa
    Eigen::Vector4f surface_centroid;
    compute3DCentroid(*surface_pc, surface_centroid);
    ROS_INFO("Creando cliente para '%s'",LOOKAT_SRV_NAME.c_str());
    ros::ServiceClient lookat_client = global_nh->serviceClient<memoria::LookAt>(LOOKAT_SRV_NAME);
    memoria::LookAt lookat_srv;
    memoria::LookAtMsg lookatmsg;
    lookatmsg.frame_id = BASE_FRAME;
    lookatmsg.rotate = 0;
    lookatmsg.vector.x = surface_centroid[0];
    lookatmsg.vector.y = surface_centroid[1];
    lookatmsg.vector.z = surface_centroid[2];
    lookat_srv.request.lookatmsg = lookatmsg;
    ROS_INFO("Mirando a la mesa");
    if (not lookat_client.call(lookat_srv)){
        ROS_ERROR("Error al llamar al servicio 'look_at'");
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
    global_nh = &nh;
    
    // Conectar a los servicios
    ROS_INFO("Creando cliente para '%s'",SEARCH_SURFACE_SRV_NAME.c_str());
    search_surface_client = nh.serviceClient<memoria::SearchSurface>(SEARCH_SURFACE_SRV_NAME);
    ROS_INFO("Creando cliente para '%s'",GO_TO_POSE_SRV_NAME.c_str());
    go_to_pose_client = nh.serviceClient<memoria::GoToPose>(GO_TO_POSE_SRV_NAME);

    // Publicar dummies
    aux_pointcloud_pub = nh.advertise<PointCloud<PointXYZ> > ("plano_dummy",1);
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
    // Nube que almacenará la superficie encontrada, relativa al frame odométrico.
    PointCloud<PointXYZ>::Ptr surface_pc(new PointCloud<PointXYZ>());
    searchSurface(surface_pc);
    ROS_INFO("Iniciando desplazamiento hacia superficie");
    moveToSurface(surface_pc);
    ROS_INFO("Iniciando cálculo de pose para objeto");
    getPlacingPose();
    /*ROS_INFO("Iniciando placing");
    placeObject();
    // ROS_INFO("Placing finalizado con éxito.");*/

    return 0;
}
/*
    TODO:
        [DONE]- Reparar rotación al llegar a la mesa
            Se implementó forma manual de calcular Yaw a partir de Quaternion. getAngle retornaba cosas positivas siempre.
        [DONE]- Reparar problema "superficie demasiado inclinada" cuando no deberia pasar
            A veces se retorna pitch positivo siendo que realmente es negativo. Revisar implementación. Por ahora se parcha tomando -1*abs(pitch).
        [DONE]- Rotar primero, luego avanzar
            La rotación genera mucho error. 
            1) Avanza
            2) Rota
            3) Avanza un último pedacito
        [DONE]- Cabeza debe mirar inicialmente un poco más cerca
            Hardcodeo en LookAt
        [DONE]- Reparar dirección de pose al lado de la mesa
            Cálculo de orientación reparado
        [DONE]- Reparar el que se pase de largo al ir a la pose de la mesa.
            Corrección hardcodeada en GoToPose
        [DONE]- Reparar caso en que distancia a punto más cercano es menor a correcciones
            Al parecer se repara solo por implementación
        [DONE]- Actualizar diagramas de flujo con arquitectura anterior
        [DONE]- Terminar moveToSurface mirando centroide de superficie
        - Implementar servicio (o método) getPlacingPose
        - Implementar placeObject
        - Cambiar diagrama de flujo en parte de Tuck Arms. Debe tuckearse sólo el brazo que no tiene el objeto. Esto debe saberse de antemano.
        - Considerar altura del torso al llegar a la superficie
        [DONE]- Considerar cambiar servicios por métodos en una clase auxiliar

    Propuestas para el futuro
        - Búsqueda más inteligente de superficie: Si encuentro un pedazo plano pero chico, puedo mirar más hacia esa parte, probablemente vi una punta de una mesa y puedo encontrar el resto.


 */