/*
 Este es el nodo principal de la memoria.
 Este programa se preocupa de hacer todo lo necesario para lograr que el robot posicione el objeto
 */
#include <string>
#include <vector>
#include <csignal>
#include <ros/ros.h>
#include <algorithm>
// Mensajes
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
// Propios
#include "RobotDriver/RobotDriver.h"
#include "Util/Util.h"
#include "PlacedObject/PlacedObject.h"
#include "PlacingSurface/PlacingSurface.h"

using namespace std;
using namespace pcl;

// CONSTANTES

// VARIABLES GLOBALES
RobotDriver *r_driver;
char grasp_arm;
PlacedObject *the_object;
PlacingSurface *the_surface;

// Auxiliares y cosas para visualizar
ros::Publisher object_pc_pub;
ros::Publisher gripper_pc_pub;
ros::Publisher surface_pc_pub;


ros::Publisher closest_point_pub;
// ros::Publisher surface_centroid_pub;
ros::Publisher closest_pose_pub;
ros::Publisher stable_pose_pub;
ros::Publisher surface_pose_pub;


// Pre-declaración de métodos
void endProgram(int retcode);
void signalHandler(int signum);
bool searchSurface(PointCloud<PointXYZ>::Ptr &cloud_out);
bool moveToSurface(PointCloud<PointXYZ>::Ptr cloud);
bool getStableObjectPose(geometry_msgs::PoseStamped &pose_out);
bool scanGripper(PointCloud<PointXYZ>::Ptr &object_out, PointCloud<PointXYZ>::Ptr &gripper_out);

// MÉTODOS
// Handlers finalización
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

// Métodos memoria


/**
 * getPlacedObject hace las tareas necesarias para escanear el gripper y obtener una representación del objeto y del gripper.
 *                     El resultado es un objeto (c++) con la representación del objeto (físico) internamente definido, con su pose estable y todo.
 * @param  the_blank_object Objeto (c++) que representa al objeto (físico)
 * @return                  True en success, false en error.
 */
bool getPlacedObject(){
    bool gotopose_ok;
    PointCloud<PointXYZ>::Ptr gripper_out (new PointCloud<PointXYZ>());
    PointCloud<PointXYZ>::Ptr object_out (new PointCloud<PointXYZ>());
    // Mover gripper no activo a posición donde no moleste
    ROS_INFO("PLACE: Quitar del campo visual al gripper inactivo");
    geometry_msgs::PoseStamped tuck_pose;
    tuck_pose.pose.position.x = Util::tuck_position[0];
    tuck_pose.pose.position.y = (grasp_arm == 'l' ? Util::tuck_position[1]*-1 : Util::tuck_position[1]);
    tuck_pose.pose.position.z = Util::tuck_position[2];
    tuck_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(Util::tuck_orientation[0], Util::tuck_orientation[1], Util::tuck_orientation[2]);
    tuck_pose.header.frame_id = Util::BASE_FRAME;
    if (grasp_arm == 'l')
        gotopose_ok = r_driver->rgripper->goToPose(tuck_pose);
    else
        gotopose_ok = r_driver->lgripper->goToPose(tuck_pose);
    if (not gotopose_ok){
        ROS_ERROR("PLACE: No se pudo tuckear brazo %s", grasp_arm == 'l' ? "derecho" : "izquierdo");
        return false;
    }
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
    // Poner gripper en pose inicial
    scan_orientation[0] += Util::SCAN_ROLL_DELTA;
    scan_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(scan_orientation[0], scan_orientation[1], scan_orientation[2]);
    string GRIPPER_FRAME = (grasp_arm == 'l' ? "l" : "r") + Util::GRIPPER_FRAME_SUFFIX;
    if (grasp_arm == 'l')
        gotopose_ok = r_driver->lgripper->goToPose(scan_pose);
    else
        gotopose_ok = r_driver->rgripper->goToPose(scan_pose);
    if (not gotopose_ok){
        ROS_ERROR("PLACE: No se pudo ir a la pose especificada");
        return false;
    }
    // Mirar al gripper
    r_driver->head->lookAt(Util::BASE_FRAME, Util::scan_position[0], Util::scan_position[1], Util::scan_position[2]);
    ros::Duration(Util::GRIPPER_STABILIZE_TIME).sleep();
    // Contenedores para objetos creados en el ciclo
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>()), 
                              cloud_near(new PointCloud<PointXYZ>()),
                              cloud_subsampled(new PointCloud<PointXYZ>()),
                              cloud_gripper(new PointCloud<PointXYZ>());
    vector<PointCloud<PointXYZ> > cloud_scans;
    Eigen::Matrix4f transformation;
    // Scanear
    while ( 1 ){
        // Pedir nube de kinect
        cloud = r_driver->sensors->kinect->getNewCloud();
        ROS_INFO("PLACE: nube recibida desde kinect: %d puntos, frame: %s", (int)cloud->points.size(), cloud->header.frame_id.c_str());
        ROS_INFO("PLACe: Primer punto está en (%f, %f, %f)", cloud->points[0].x, cloud->points[1].y, cloud->points[2].z);
        
        // obtener transformación de kinect a wrist
        transformation = Util::getTransformation(Util::KINECT_FRAME, GRIPPER_FRAME);
        // Eliminar puntos lejanos (respecto a kinect)
        PassThrough<PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.6 - Util::SCAN_PASSTHROUGH_Z/2.0, 0.6 + Util::SCAN_PASSTHROUGH_Z/2.0); // HARDCODEADO. FORMA CORRECTA ES CON TF?
        pass.filter(*cloud_near);
        // submuestrear
        cloud_subsampled = Util::subsampleCloud(cloud_near, Util::SCAN_LEAFSIZE);
        // Transformar y agregar a scans
        transformPointCloud(*cloud_subsampled, *cloud_gripper, transformation);
        cloud_gripper->header.frame_id = GRIPPER_FRAME;
        cloud_scans.push_back(*cloud_gripper);

        // Si ya escaneamos todas las perspectivas, juntarlas y retornar
        scan_orientation[0] += Util::SCAN_ROLL_DELTA;
        if (scan_orientation[0] > 2*Util::PI){
            ROS_INFO("PLACE: Se obtuvieron todos los scans. Uniendo...");
            PointCloud<PointXYZ>::Ptr merged(new PointCloud<PointXYZ>());
            for (int i=0; i<cloud_scans.size(); i++){
                *merged += cloud_scans[i];
            }
            merged->header.frame_id = GRIPPER_FRAME;
            if (merged->points.size() == 0){
                ROS_ERROR("PLACE: El scanneo del gripper retornó 0 puntos!");
                return false;
            }
            ROS_INFO("PLACE: Union resulto en nube de %d puntos", (int)merged->points.size());
            // Filtrar gripper_out
            ROS_INFO("PLACE: Filtrando scans");
            if (not Util::gripperFilter(merged, object_out, gripper_out)){
                ROS_ERROR("PLACE: Algo ocurrio al intentar filtrar el gripper");
                return false;
            }
            ROS_INFO("PLACE: Objeto: %d puntos. Gripper: %d puntos.", (int)object_out->points.size(), (int)gripper_out->points.size());
            object_out->header.frame_id = GRIPPER_FRAME;
            gripper_out->header.frame_id = GRIPPER_FRAME;
            the_object->setClouds(object_out, gripper_out);
            // Obtener pose estable y otras características
            the_object->computeStablePose();
            return true;
        }
        ROS_INFO("PLACE: Posicionandose psra siguiente scan");
        scan_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(scan_orientation[0], scan_orientation[1], scan_orientation[2]);
        if (grasp_arm == 'l')
            gotopose_ok = r_driver->lgripper->goToPose(scan_pose);
        else
            gotopose_ok = r_driver->rgripper->goToPose(scan_pose);
        if (not gotopose_ok){
            ROS_ERROR("PLACE: No se pudo ir a la pose especificada");
            return false;
        }
        ros::Duration(Util::GRIPPER_STABILIZE_TIME).sleep();
    }
}

/**
 * getPlacingSurface Busca superficie tomando en cuenta características del objeto a posicionar. Almacena superficie en objeto (c++) que la representa
 * @param  the_blank_surface Superficie nueva a inicializar
 * @param  the_object        Objeto previamente encontrado
 * @return                   True en success, False en error
 */
bool getPlacingSurface(){
    bool gotopose_ok;
    geometry_msgs::PoseStamped surface_normal;
    geometry_msgs::PointStamped surface_centroid;
    // Mover gripper con objeto a posición donde no moleste
    ROS_INFO("PLACE: Quitar del campo visual al gripper con objeto");
    geometry_msgs::PoseStamped tuck_pose_active;
    tuck_pose_active.pose.position.x = Util::active_gripper_starting_position[0];
    tuck_pose_active.pose.position.y = (grasp_arm == 'r' ? Util::active_gripper_starting_position[1]*-1 : Util::active_gripper_starting_position[1]);
    tuck_pose_active.pose.position.z = Util::active_gripper_starting_position[2];
    tuck_pose_active.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(Util::active_gripper_starting_orientation[0], Util::active_gripper_starting_orientation[1], Util::active_gripper_starting_orientation[2]);
    tuck_pose_active.header.frame_id = Util::BASE_FRAME;
    if (grasp_arm == 'l')
        gotopose_ok = r_driver->lgripper->goToPose(tuck_pose_active);
    else
        gotopose_ok = r_driver->rgripper->goToPose(tuck_pose_active);
    if (not gotopose_ok){
        ROS_ERROR("PLACE: No se pudo tuckear brazo %s", grasp_arm == 'l' ? "derecho" : "izquierdo");
        return false;
    }
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
        ROS_INFO("PLACE: Rotando a yaw = %f", Util::toGrad(yaw));
        // Rotar cabeza
        r_driver->head->rotate(Util::BASE_FRAME, yaw);
        ros::Duration(Util::KINECT_STABILIZE_TIME).sleep();
        // Obtener nube de puntos desde kinect
        ROS_INFO("PLACE: Esperando nube de puntos...");
        cloud = r_driver->sensors->kinect->getNewCloud();
        ROS_INFO("PLACE: Nube de %d puntos recibida", (int)cloud->points.size());
        // Submuestrear nube
        ROS_INFO("PLACE: Submuestreando...");
        cloud_subsampled = Util::subsampleCloud(cloud, Util::SUBSAMPLE_LEAFSIZE);
        ROS_INFO("PLACE: %d puntos tras submuestreo", (int)cloud_subsampled->points.size());
        // Buscar un plano adecuado
        ROS_INFO("PLACE: Buscando superficie para placing...");
        if (Util::searchPlacingSurface(cloud_subsampled, cloud_surface, surface_normal, surface_centroid, 0.3, 0.8, Util::DEFAULT_DESIRED_PITCH)){
            ROS_INFO("PLACE: Encontrada");

            ROS_INFO("PLACE: Refinando superficie");
            // Repasar superficie:
            //      1) Obtener centroide
            //      2) Mirar centroide
            //      3) Buscar superficie denuevo
            //      4) Iterar N veces
            for (int i=0; i<Util::SURFACE_REFINING_ITERATIONS; i++){
                ROS_INFO("PLACE: Iteracion %d de %d", i, Util::SURFACE_REFINING_ITERATIONS);
                // Obtener centroide
                geometry_msgs::Point centroid = Util::getCloudCentroid(cloud_surface);
                // Mirar centroide
                r_driver->head->lookAt(cloud_surface->header.frame_id, centroid.x, centroid.y, centroid.z);
                ros::Duration(Util::KINECT_STABILIZE_TIME).sleep();
                // Buscar superficie denuevo
                cloud = r_driver->sensors->kinect->getNewCloud();
                cloud_subsampled = Util::subsampleCloud(cloud, Util::SUBSAMPLE_LEAFSIZE);
                if (not Util::searchPlacingSurface(cloud_subsampled, cloud_surface, surface_normal, surface_centroid, centroid.z - Util::SURFACE_REFINING_THRESHOLD, centroid.z + Util::SURFACE_REFINING_THRESHOLD, Util::DEFAULT_DESIRED_PITCH)){
                    ROS_ERROR("PLACE: Superficie no pudo ser refinada. Este error no debiera ocurrir. Abortando");
                    return false;
                }
            }
            ROS_INFO("PRE ASIGNACION cloud");
            the_surface->setCloud(cloud_surface);
            ROS_INFO("PRE ASIGNACION normal");
            the_surface->setNormal(surface_normal);
            ROS_INFO("PRE ASIGNACION centroid");
            the_surface->setCentroid(surface_centroid);
            ROS_INFO("POST ASIGNACION");
            return true;
        }
        ROS_INFO("PLACE: No encontrada. Iterando...");
        yaw += yaw_step;
    }
    // Si no se encontró, buscar hacia el otro lado
    // Subir la mirada e iterar
    r_driver->head->lookAt(Util::BASE_FRAME, 0, 2, 1.0);
    while (yaw >= min_yaw){
        ROS_INFO("PLACE: Rotando a yaw = %f", Util::toGrad(yaw));
        // Rotar cabeza
        r_driver->head->rotate(Util::BASE_FRAME, yaw);
        ros::Duration(Util::KINECT_STABILIZE_TIME).sleep();
        // Obtener nube de puntos desde kinect
        ROS_INFO("PLACE: Esperando nube de puntos...");
        cloud = r_driver->sensors->kinect->getNewCloud();
        ROS_INFO("PLACE: Nube de %d puntos recibida", (int)cloud->points.size());
        // Submuestrear nube
        ROS_INFO("PLACE: Submuestreando...");
        cloud_subsampled = Util::subsampleCloud(cloud, Util::SUBSAMPLE_LEAFSIZE);
        ROS_INFO("PLACE: %d puntos tras submuestreo", (int)cloud_subsampled->points.size());
        // Buscar un plano adecuado
        ROS_INFO("PLACE: Buscando superficie para placing...");
        if (Util::searchPlacingSurface(cloud_subsampled, cloud_surface, surface_normal, surface_centroid, 0.3, 0.8, Util::DEFAULT_DESIRED_PITCH)){
            ROS_INFO("PLACE: Encontrada");
            the_surface->setCloud(cloud_surface);
            the_surface->setNormal(surface_normal);
            the_surface->setCentroid(surface_centroid);
            return true;
        }
        ROS_INFO("PLACE: No encontrada. Iterando...");
        yaw -= yaw_step;
    }
    return false;
}

bool moveToSurface(PointCloud<PointXYZ>::Ptr cloud){
    bool gotopose_ok;
    // Mover otro gripper a posición donde no moleste
    geometry_msgs::PoseStamped tuck_pose;
    tuck_pose.pose.position.x = Util::tuck_position[0];
    tuck_pose.pose.position.y = (grasp_arm == 'l' ? Util::tuck_position[1]*-1 : Util::tuck_position[1]);
    tuck_pose.pose.position.z = Util::tuck_position[2];
    tuck_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(Util::tuck_orientation[0], Util::tuck_orientation[1], Util::tuck_orientation[2]);
    tuck_pose.header.frame_id = Util::BASE_FRAME;
    if (grasp_arm == 'l')
        gotopose_ok = r_driver->rgripper->goToPose(tuck_pose);
    else
        gotopose_ok = r_driver->lgripper->goToPose(tuck_pose);
    if (not gotopose_ok){
        ROS_ERROR("PLACE: No se pudo tuckear brazo %s", grasp_arm == 'l' ? "derecho" : "izquierdo");
        return false;
    }
    // Mover objeto a pose de scanning, para no chocar con las cosas
    geometry_msgs::PoseStamped scan_pose;
    scan_pose.header.frame_id = Util::BASE_FRAME;
    scan_pose.pose.position.x = Util::scan_position[0];
    scan_pose.pose.position.y = Util::scan_position[1];
    scan_pose.pose.position.z = Util::scan_position[2];
    scan_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(Util::scan_orientation[0], Util::scan_orientation[1], Util::scan_orientation[2]);
    if (grasp_arm == 'l')
        gotopose_ok = r_driver->lgripper->goToPose(scan_pose);
    else
        gotopose_ok = r_driver->rgripper->goToPose(scan_pose);
    if (not gotopose_ok){
        ROS_ERROR("PLACE: No se pudo levantar gripper activo para desplazarse");
        return false;
    }
    // Transformar la nube de Odom a Base;
    Eigen::Matrix4f transformation = Util::getTransformation(cloud->header.frame_id, Util::BASE_FRAME);
    PointCloud<PointXYZ>::Ptr cloud_base (new PointCloud<PointXYZ>());
    transformPointCloud(*cloud, *cloud_base, transformation);
    cloud_base->header.frame_id = Util::BASE_FRAME;
    // Elegir punto más cercano a la superficie (respecto a la base!)
    geometry_msgs::PointStamped closest_point;
    float closest_point_distance;
    Util::getClosestPoint(cloud_base, closest_point, closest_point_distance);
    closest_point_pub.publish(closest_point);
    // Obtener una pose adecuada para llegar a ese punto
    // la pose se encuentra un poquito más atrás que el punto.
    ROS_INFO("PLACE: Punto más cercano está a %fm", closest_point_distance);
    if (closest_point_distance > Util::ROBOT_FRONT_MARGIN){
        ROS_INFO("PLACE: Debo acercarme");
        tf::Vector3 closest_point_floor_vector (closest_point.point.x, closest_point.point.y, 0);
        float closest_point_floor_distance = closest_point_floor_vector.length();
        closest_point_floor_vector.normalize();
        closest_point_floor_vector*=closest_point_floor_distance - Util::ROBOT_FRONT_MARGIN;
        ROS_INFO("PLACE: Me ubicaré a %fm de la pose actual", closest_point_floor_vector.length());
        geometry_msgs::PoseStamped closest_pose;
        closest_pose.header.frame_id = Util::BASE_FRAME;
        closest_pose.pose.position.x = closest_point_floor_vector.x();
        closest_pose.pose.position.y = closest_point_floor_vector.y();
        closest_pose.pose.position.z = closest_point_floor_vector.z();
        closest_pose.pose.orientation = Util::coefsToQuaternionMsg(closest_point_floor_vector.x(), closest_point_floor_vector.y(), 0);
        // Ver pose
        closest_pose_pub.publish(closest_pose);
        ROS_INFO("PLACE: Yendo a la pose...");
        if (not r_driver->base->goToPose(closest_pose)){
            ROS_ERROR("PLACE: No se pudo ir a la pose");
            return false;
        }
        ROS_INFO("PLACE: Llegué a la pose");
    }
    ROS_INFO("Mirando al centroide de la superficie");
    // Nos movimos: actualizar entonces la transformación entre odom y base
    transformation = Util::getTransformation(cloud->header.frame_id, Util::BASE_FRAME);
    // Transformar superficie
    transformPointCloud(*cloud, *cloud_base, transformation);
    // Obtener centroide (respecto a la base)
    geometry_msgs::Point centroid = Util::getCloudCentroid(cloud_base);
    // Mirar hacia el centroide
    r_driver->head->lookAt(Util::BASE_FRAME, centroid.x, centroid.y, centroid.z);
    return true;    
}

/**
 * placeObject         Toma la pose estable del objeto, relativo al gripper, y la nube de puntos de la superficie
 *                     encontrada, para buscar en la superficie algun lugar donde poder poner el objeto. Cuando encuentra
 *                     una posición posible, obtiene una transformación entre la pose estable del objeto y la del spot encontrado, 
 *                     luego procede a calcular la pose del gripper para llegar a ese punto, y evalúa su factibilidad.
 *                     Si es factible, procede a soltar el objeto y retirar el gripper.
 *                     Se implementa además una corrección al método de obtención de transformación, que obtiene sentido erróneo a veces.
 * @param  stable_pose Pose más estable del objeto, RELATIVA AL GRIPPER
 * @param  surface_pc  Superficie de placing encontrada antes, RELATIVA A ODOM. Se ignorarán partes no vistas (no se extrapolará en el plano)
 * @param  surface_normal_pose Pose RELATIVA A ODOM centrada en el centroide de la superficie, y orientada normal a la superficie.
 * @return             True en éxito, false en caso contrario
 */
bool placeObject(){
    ros::NodeHandle nh_;
    ros::Publisher gripper_pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("gripper_pose",1);
    ros::Publisher gripper_future_pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("gripper_future_pose",1);

    // Eliminar puntos donde no puede ocurrir el placing
    PointIndices::Ptr placing_points = Util::getFactiblePlacingPointsIndices((the_surface->cloud), the_surface->normal, the_object->base_area);

    // // test de correctitud de transformación 
    // ros::Publisher test_pose_ini_pub = nh_.advertise<geometry_msgs::PoseStamped>("pose_ini_unitv", 1);
    // ros::Publisher test_pose_end_pub = nh_.advertise<geometry_msgs::PoseStamped>("pose_end_unitv", 1);
    // ros::Publisher test_pose_transformed_end_pub = nh_.advertise<geometry_msgs::PoseStamped>("pose_end_transformed", 1);
    // ros::Publisher test_pose_transformed_end_quat_pub = nh_.advertise<geometry_msgs::PoseStamped>("pose_end_transformed_quat", 1);


    // Obtener transformación, transformar pose y plano a baseframe
    geometry_msgs::PoseStamped stable_pose_base;
    stable_pose_base.header.frame_id = Util::BASE_FRAME;
    Eigen::Matrix4f gripper_to_base_tf = Util::getTransformation(the_object->stable_pose.header.frame_id, Util::BASE_FRAME);
    stable_pose_base.pose = Util::transformPose(the_object->stable_pose.pose, gripper_to_base_tf);
    Eigen::Matrix4f odom_to_base_tf = Util::getTransformation(Util::ODOM_FRAME, Util::BASE_FRAME);
    PointCloud<PointXYZ>::Ptr surface_base_pc (new PointCloud<PointXYZ>());
    transformPointCloud(*(the_surface->cloud), *surface_base_pc, odom_to_base_tf);
    // Iterar (hasta éxito o hasta máximo de intentos)
    // Crear arreglo de 0 a (the_surface->cloud)->points.size()
    int pc_indices[(int)(the_surface->cloud)->points.size()];
    for (int i=0; i<(int)(the_surface->cloud)->points.size(); i++)
        pc_indices[i] = i;
    // Desordenar indices
    random_shuffle(&pc_indices[0], &pc_indices[(int)(the_surface->cloud)->points.size()-1]);
    for (int i=0; i<(int)(the_surface->cloud)->points.size(); i++){
        ROS_INFO("PLACE: Mostrando posible punto de placing (indice %d de %d)", i, (int)(the_surface->cloud)->points.size());
        // Construyo una posible pose, basado en puntos de la superficie y la normal transformada
        geometry_msgs::PoseStamped new_surface_pose;
        new_surface_pose.header.frame_id = Util::BASE_FRAME;
        new_surface_pose.pose.position.x = (the_surface->cloud)->points[pc_indices[i]].x;
        new_surface_pose.pose.position.y = (the_surface->cloud)->points[pc_indices[i]].y;
        new_surface_pose.pose.position.z = (the_surface->cloud)->points[pc_indices[i]].z;
        new_surface_pose.pose.orientation = (Util::transformPose(the_surface->normal.pose, odom_to_base_tf)).orientation;
        surface_pose_pub.publish(new_surface_pose);
        surface_pose_pub.publish(new_surface_pose);
        surface_pose_pub.publish(new_surface_pose);
        // obtener transformación pose estable a pose alcanzable
        
        geometry_msgs::PoseStamped gripper_pose;
        if (grasp_arm == 'l')
            gripper_pose = r_driver->lgripper->getCurrentPose();
        else
            gripper_pose = r_driver->rgripper->getCurrentPose();
        ROS_INFO("PLACE: Transformando pose gripper a baseframe");
        geometry_msgs::PoseStamped gripper_pose_base;
        gripper_pose_base.pose = Util::transformPose(gripper_pose.pose, Util::getTransformation(gripper_pose.header.frame_id, Util::BASE_FRAME));
        gripper_pose_base.header.frame_id = Util::BASE_FRAME;
        gripper_pose_pub.publish(gripper_pose_base);
        gripper_pose_pub.publish(gripper_pose_base);
        gripper_pose_pub.publish(gripper_pose_base);
        ROS_INFO("PLACE: Calculando y mostrando futura posible pose del gripper");
        geometry_msgs::PoseStamped gripper_future_pose;
        gripper_future_pose.header.frame_id = Util::BASE_FRAME;
        Eigen::Vector3f pose_ini_orientation = Util::quaternionMsgToVector(stable_pose_base.pose.orientation);
        Eigen::Vector3f pose_end_orientation = Util::quaternionMsgToVector(new_surface_pose.pose.orientation);
        // ROTACION CON QUATERNION
        // Obtener quaternion de rotación entre pose inicial y final del objeto
        Eigen::Quaternionf rotation_q = Util::getQuaternionBetweenVectors(pose_ini_orientation, pose_end_orientation);
        // Rotar orientación y posición del gripper
        Eigen::Vector3f gripper_p (gripper_pose_base.pose.position.x, gripper_pose_base.pose.position.y, gripper_pose_base.pose.position.z);
        Eigen::Vector3f gripper_q = Util::quaternionMsgToVector(gripper_pose_base.pose.orientation);
        Eigen::Vector3f object_pose_ini (stable_pose_base.pose.position.x, stable_pose_base.pose.position.y, stable_pose_base.pose.position.z);
        Eigen::Vector3f object_pose_end (new_surface_pose.pose.position.x, new_surface_pose.pose.position.y, new_surface_pose.pose.position.z);
        // Usar valores de pose que manda para transformar gripper_pose_base
        gripper_p -= object_pose_ini;
        Eigen::Quaternionf gripper_p_quat = Util::eigenVectorToQuaternion(gripper_p);
        gripper_p_quat = rotation_q*gripper_p_quat*rotation_q.inverse();
        Eigen::Quaternionf gripper_q_quat = Util::eigenVectorToQuaternion(gripper_q);
        gripper_q_quat = rotation_q*gripper_q_quat*rotation_q.inverse();
        gripper_q_quat.normalize();
        gripper_q = gripper_q_quat.vec();
        gripper_p = gripper_p_quat.vec() + object_pose_end;
        gripper_p.z() += Util::PLACING_Z_MARGIN;
        
        // Rotar pose alrededor de la normal de la pose tentativa
        if (gripper_q[0] < 0){
            ROS_INFO("PLACE: Pose apunta hacia el robot... rotando");
            Eigen::AngleAxis<float> rotate(Util::PI, Util::quaternionMsgToVector(new_surface_pose.pose.orientation));
            gripper_p -= object_pose_end;
            gripper_p = rotate*gripper_p;
            gripper_p += object_pose_end;
            gripper_q = rotate*gripper_q;
        }
        gripper_future_pose.pose.position.x = gripper_p[0];
        gripper_future_pose.pose.position.y = gripper_p[1];
        gripper_future_pose.pose.position.z = gripper_p[2];
        gripper_future_pose.pose.orientation = Util::coefsToQuaternionMsg(gripper_q[0], gripper_q[1], gripper_q[2]);
        // Publicar
        gripper_future_pose_pub.publish(gripper_future_pose);
        gripper_future_pose_pub.publish(gripper_future_pose);
        gripper_future_pose_pub.publish(gripper_future_pose);
        // Intentar ir a la pose
        bool gotopose_ok;
        if (grasp_arm == 'l')
            gotopose_ok = r_driver->lgripper->goToPose(gripper_future_pose);
        else
            gotopose_ok = r_driver->rgripper->goToPose(gripper_future_pose);
        if (gotopose_ok){
            // Soltar objeto
            // Quitar gripper
            // El gripper retrocederá una distancia determinada, en la misma dirección que su orientación
            geometry_msgs::PoseStamped gripper_backoff_pose = gripper_future_pose;
            Eigen::Vector3f backoff_position = gripper_p - Util::PLACING_BACKOFF_DISTANCE*(gripper_q.normalized());
            gripper_backoff_pose.pose.position.x = backoff_position[0];
            gripper_backoff_pose.pose.position.y = backoff_position[1];
            gripper_backoff_pose.pose.position.z = backoff_position[2];
            if (grasp_arm == 'l'){
                ROS_INFO("PLACE: Abriendo gripper izquierdo");
                r_driver->lgripper->setOpening(1, -1);
                ros::Duration(1.0).sleep();
                ROS_INFO("PLACE: Retrocediendo gripper");
                r_driver->lgripper->goToPose(gripper_backoff_pose);
                ROS_INFO("PLACE: Cerrando gripper izquierdo");
                r_driver->lgripper->setOpening(0, 10);
            }
            else{
                ROS_INFO("PLACE: Abriendo gripper derecho");
                r_driver->rgripper->setOpening(1, -1);
                ros::Duration(1.0).sleep();
                ROS_INFO("PLACE: Retrocediendo gripper");
                r_driver->rgripper->goToPose(gripper_backoff_pose);
                ROS_INFO("PLACE: Cerrando gripper derecho");
                r_driver->rgripper->setOpening(0, 100);
            }
            return true;
        }
        // END TEST

        ros::Duration(0.3).sleep();
    }
    return false;
    
}
void place(moveit::planning_interface::MoveGroup &group, geometry_msgs::PoseStamped placing_pose){
    vector<moveit_msgs::PlaceLocation> loc;
    moveit_msgs::PlaceLocation g;
    g.place_pose = placing_pose;

    g.pre_place_approach.direction.vector.z = -1.0;
    g.post_place_retreat.direction.vector.x = -1.0;
    g.post_place_retreat.direction.header.frame_id = Util::BASE_FRAME;
    stringstream gripper_frame_id;
    gripper_frame_id << grasp_arm << "_wrist_roll_link";
    g.pre_place_approach.direction.header.frame_id = gripper_frame_id.str();
    g.pre_place_approach.min_distance = 0.1;
    g.pre_place_approach.desired_distance = 0.2;
    g.post_place_retreat.min_distance = 0.1;
    g.post_place_retreat.desired_distance = 0.25;
    stringstream gripper_joint;
    gripper_joint << grasp_arm << "_gripper_joint";
    g.post_place_posture.joint_names.resize(1, gripper_joint.str());
    g.post_place_posture.points.resize(1);
    g.post_place_posture.points[0].positions.resize(1);
    g.post_place_posture.points[0].positions[0] = 1;

    loc.push_back(g);
    group.setSupportSurfaceName("mesa");

    // Agregar path constraints
    // 
    group.setPlannerId("RRTConnectkConfigDefault");
    group.place("part", loc);
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
    // Para terminar con ctrl+c
    signal(SIGINT, signalHandler);
    // Iniciando robot driver
    ROS_INFO("PLACE: Iniciando RobotDriver");
    r_driver = new RobotDriver();
    ROS_INFO("PLACE: RobotDriver Creado e iniciado");

    //   ZONA DE PRUEBAS

    surface_pc_pub = nh.advertise<PointCloud<PointXYZ> >("surface", 1);
    object_pc_pub = nh.advertise<PointCloud<PointXYZ> >("object_pc", 1);
    gripper_pc_pub = nh.advertise<PointCloud<PointXYZ> >("gripper_pc", 1);

    closest_point_pub = nh.advertise<geometry_msgs::PointStamped>("closest_point", 1);
    // surface_centroid_pub = nh.advertise<geometry_msgs::PointStamped>("surface_centroid", 1);
    closest_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("closest_pose", 1);
    stable_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("stable_pose", 1);
    surface_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("surface_pose", 1);

    ros::Publisher planning_pub = nh.advertise<moveit_msgs::PlanningScene>("/move_group/monitored_planning_scene", 1);
    ros::Publisher aco_pub = nh.advertise<moveit_msgs::AttachedCollisionObject>("/attached_collision_object", 10);
    ros::Publisher co_pub = nh.advertise<moveit_msgs::CollisionObject>("/collision_object", 10);
    

    ros::Duration(2).sleep();

// TESTEANDO NUEVO ORDEN: SCAN -> POSE ESTABLE -> SEARCH SURFACE -> PLACE

    // Borrar octomap
    // ROS_INFO("PLACE: Intentando borrar octomap");
    // moveit_msgs::PlanningScene clear_octomap;
    // planning_pub.publish(clear_octomap);
    ROS_INFO("PLACE: Attachando esfera");
    // Creando collision object
    moveit_msgs::CollisionObject co;
    // Frame de referencia para la pose
    co.header.frame_id = "l_gripper_tool_frame";
    // co.header.stamp = ros::Time(0);
    // Nombre del objeto
    co.id = "my_collision_object";
    // Creando shape
    shape_msgs::SolidPrimitive shape;
    shape.type = shape.SPHERE;
    const float shape_size = 0.3;
    shape.dimensions.push_back(shape_size); // x
    shape.dimensions.push_back(shape_size); // y
    shape.dimensions.push_back(shape_size); // z 
    // Definiendo pose
    geometry_msgs::Pose co_pose;
    co_pose.position.x = co_pose.position.y = co_pose.position.z = co_pose.orientation.x = co_pose.orientation.y = co_pose.orientation.z = 0;
    co_pose.orientation.w = 1;
    co.primitives.push_back(shape);
    co.primitive_poses.push_back(co_pose);

    // Creando attached collision object
    moveit_msgs::AttachedCollisionObject aco;
    // Se attacha al gripper
    aco.link_name = "l_wrist_roll_link";

    aco.touch_links.push_back("l_gripper_r_finger_tip_link"); // redundante
    aco.touch_links.push_back("l_gripper_r_finger_link"); // redundante
    aco.touch_links.push_back("l_gripper_l_finger_tip_link"); // redundante
    aco.touch_links.push_back("l_gripper_l_finger_link"); // redundante
    aco.touch_links.push_back("l_wrist_roll_link"); // redundante
    aco.touch_links.push_back("l_wrist_flex_link");
    aco.touch_links.push_back("l_forearm_roll_link");
    aco.touch_links.push_back("l_elbow_flex_link");
    aco.touch_links.push_back("l_upper_arm_roll_link");
    aco.touch_links.push_back("base_link");

    // links que salen en RViz y no en gazebo
    aco.touch_links.push_back("l_forearm_link");
    aco.touch_links.push_back("l_gripper_motor_accelerometer_link");
    aco.touch_links.push_back("l_gripper_palm_link");
    aco.touch_links.push_back("l_upper_arm_link");


    
    


    // Primero lo removeremos
    co.operation = co.REMOVE;
    // Removiendo objeto
    aco.object = co;
    ROS_INFO("Removiendo '%s'", co.id.c_str());
    aco_pub.publish(aco);
    // Ahora lo agregamos
    ros::Duration(1).sleep();
    co.operation = co.ADD;
    aco.object = co;
    ROS_INFO("Agregando '%s'", co.id.c_str());
    aco_pub.publish(aco);
    // collision_pub.publish(aco);
    // collision_pub.publish(aco);



    the_object = new PlacedObject();
    the_surface = new PlacingSurface();
    // Obtener objeto con su pose estable
    if (not getPlacedObject()){
        ROS_ERROR("No se pudo obtener el objeto");
        endProgram(1);
    }
    ROS_INFO("Objeto obtenido. Obteniendo superficie");
    object_pc_pub.publish(the_object->object_pc);
    object_pc_pub.publish(the_object->object_pc);
    object_pc_pub.publish(the_object->object_pc);
    gripper_pc_pub.publish(the_object->gripper_pc);
    gripper_pc_pub.publish(the_object->gripper_pc);
    gripper_pc_pub.publish(the_object->gripper_pc);
    // Buscar superficie según limitaciones del objeto
    if (not getPlacingSurface()){
        ROS_ERROR("No se pudo obtener superficie de placing");
        endProgram(1);
    }
    ROS_INFO("Superficie obtenida");
    // Moverse hacia superficie
    // Intentar posicionar
    ROS_INFO("Posicionando superficie");
    if (not placeObject()){
        ROS_ERROR("No se pudo posicionar objeto :(");
        endProgram(1);
    }




    // END ZONA DE PRUEBAS

    ros::Duration(1).sleep();
    ROS_INFO("PLACE: FIN");
    endProgram(0);
}


/*

    TODO:
        - Testear y reparar obtención de pose de placing
        - Revisar problema de head driver de no hacer nada a veces (lanzar timeout)
        - Refinar filtro del gripper scanner:
            1) Filtrar mejor el entorno
                Usar clustering?
            2) Reintentar poses si no funcionaron
        [DONE]- Reparar dirección de pose encontrada
            Siempre debe apuntar hacia "adentro"
        - Agregar vista desde "arriba" al scaneo
        - Al llegar a la superficie, el robot debiera girar para quedar de frente al centroide de la superficie
    opcionales:
        [DONE]- Evaluar mover método "getStablePose" de Util a este archivo.
        - Evaluar corregir vista de la superficie encontrada (una vez encontrada, mirar hacia ella, volver a buscar y corregir nube)
        - Evaluar reparar head driver para rotar pitch
        - Evaluar eliminar pose inicial del head driver al inicializarlo
        - Evaluar suavizar (o "promediar") nube merged del objeto (separado del gripper)
        - Evaluar mirar al tool en vez del inicio del gripper en el escaneo
        - Evaluar habilitar movimientos simultáneos (cabeza + gripper por ejemplo, 2 grippers...)
 */