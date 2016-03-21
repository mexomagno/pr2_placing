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
ros::Publisher test_pose_pub;
// Publicador de attached collision objects
ros::Publisher aco_pub;
ros::Publisher co_pub;

// Por ordenar
geometry_msgs::PoseStamped last_scan_pose;


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
    ROS_INFO("PLACE: Comienza adquisición de modelo del objeto");
    bool gotopose_ok;
    PointCloud<PointXYZ>::Ptr object_out (new PointCloud<PointXYZ>());
    // Mover gripper no activo a posición donde no moleste
    ROS_INFO("PLACE: Quitando del campo visual al gripper inactivo");
    if (grasp_arm == 'l')
        gotopose_ok = r_driver->rgripper->moveElsewhere();
    else
        gotopose_ok = r_driver->lgripper->moveElsewhere();
    if (not gotopose_ok){
        ROS_ERROR("PLACE: No se pudo quitar brazo %s del camino", grasp_arm == 'l' ? "derecho" : "izquierdo");
        return false;
    }
    // Proteger gripper con bola para poder moverse
    Util::enableDefaultGripperCollisions(aco_pub, co_pub, true, grasp_arm);
    // Mover gripper a posición inicial de scanning
    ROS_INFO("PLACE: Moviendo gripper activo a posición de scaneo");
    // A continuación se aplica un pequeño parche para poder rotar
    // PARCHE 1) Obtener scan position relativo a ODOM
    Eigen::Matrix4f tf = Util::getTransformation(Util::TORSO_FRAME, Util::ODOM_FRAME);
    geometry_msgs::Pose scan_pose_torso;
    scan_pose_torso.position.x = Util::scan_position[0];
    scan_pose_torso.position.y = Util::scan_position[1];
    scan_pose_torso.position.z = Util::scan_position[2];
    scan_pose_torso.orientation.w = 1;
    geometry_msgs::PoseStamped scan_pose;
    scan_pose.pose = Util::transformPose(scan_pose_torso, tf);
    scan_pose.header.frame_id = Util::ODOM_FRAME;
    // ROS_INFO("PLACE: Mover gripper activo a pose de scaneo");
    // geometry_msgs::PoseStamped scan_pose;
    // scan_pose.header.frame_id = Util::TORSO_FRAME;
    // scan_pose.pose.position.x = Util::scan_position[0];
    // scan_pose.pose.position.y = Util::scan_position[1];
    // scan_pose.pose.position.z = Util::scan_position[2];
    // Copia no constante de la orientación inicial de scanning
    float scan_orientation[3];
    scan_orientation[0] = Util::scan_orientation[0];
    scan_orientation[1] = Util::scan_orientation[1];
    scan_orientation[2] = Util::scan_orientation[2];
    // Poner gripper en pose inicial
    scan_orientation[0] += Util::SCAN_ROLL_DELTA;
    scan_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(scan_orientation[0], scan_orientation[1], scan_orientation[2]);
    string GRIPPER_FRAME = (grasp_arm == 'l' ? "l" : "r") + Util::GRIPPER_FRAME_SUFFIX;

    // Setear limite de codo para robot. Esto evita no poder girar objetos grandes
    moveit_msgs::Constraints constraint;
    moveit_msgs::JointConstraint jc;
    jc.joint_name = (grasp_arm == 'l' ? "l" : "r") + Util::ARM_ROLL_JOINT_PREFIX;
    jc.position = 0;
    jc.tolerance_below = Util::toRad(70);
    jc.tolerance_above = Util::toRad(70);
    jc.weight = 10;
    constraint.joint_constraints.push_back(jc);

    if (grasp_arm == 'l'){
        r_driver->lgripper->moveit_group_->setPathConstraints(constraint);
        gotopose_ok = r_driver->lgripper->goToPose(scan_pose);
        r_driver->lgripper->moveit_group_->clearPathConstraints();
    }
    else{
        r_driver->rgripper->moveit_group_->setPathConstraints(constraint);
        gotopose_ok = r_driver->rgripper->goToPose(scan_pose);
        r_driver->rgripper->moveit_group_->clearPathConstraints();
    }
    if (not gotopose_ok){
        ROS_ERROR("PLACE: No se pudo ir a la pose especificada");
        return false;
    }
    // Mirar al gripper
    ROS_INFO("PLACE: Mirando hacia el gripper activo");
    r_driver->head->lookAt(GRIPPER_FRAME, 0, 0, 0);
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
        ROS_INFO("PLACE: Scaneando...");
        // Quitar esfera protectora del gripper
        Util::enableDefaultGripperCollisions(aco_pub, co_pub, false, grasp_arm);
        // Pedir nube de kinect
        cloud = r_driver->sensors->kinect->getNewCloud();
        // Volver a poner esfera protectora. Evita generación de octomap
        Util::enableDefaultGripperCollisions(aco_pub, co_pub, true, grasp_arm);
        ROS_INFO("PLACE: nube recibida: %d puntos  (frame: '%s')", (int)cloud->points.size(), cloud->header.frame_id.c_str());
        if ((int)cloud->points.size() == 0){
            ROS_WARN("PLACE: Se recibieron 0 puntos... eso es raro. Ojo!");
        }
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
            // ROS_INFO("PLACE: Filtrando scans");
            // if (not Util::gripperFilter(merged, object_out, gripper_out)){
            //     ROS_ERROR("PLACE: Algo ocurrio al intentar filtrar el gripper");
            //     return false;
            // }
            object_out = merged;
            the_object->setCloud(object_out);
            ROS_INFO("PLACE: Objeto: %d puntos. Gripper: %d puntos.", (int)the_object->object_pc->points.size(), (int)the_object->gripper_pc->points.size());
            // Obtener pose estable y otras características
            the_object->computeStablePose();
            // Quitar esfera protectora
            Util::enableDefaultGripperCollisions(aco_pub, co_pub, false, grasp_arm);
            // INMEDIATAMENTE Attachar mesh del objeto como collisio object al robot
            Util::attachMeshToGripper(aco_pub, co_pub, grasp_arm, the_object->polymesh);
            last_scan_pose = scan_pose;
            ROS_INFO("PLACE: Finaliza exitosamente adquisición de objeto");
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
    ROS_INFO("PLACE: Comienza adquisición de superficie de posicionamiento...");
    bool gotopose_ok;
    geometry_msgs::PoseStamped surface_normal;
    geometry_msgs::PointStamped surface_centroid;
    // Mover gripper con objeto a posición donde no moleste
    ROS_INFO("PLACE: Quitando del campo visual al gripper activo");
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
        ROS_ERROR("PLACE: No se pudo mover al brazo %s del campo visual", grasp_arm == 'l' ? "izquierdo" : "derecho");
        return false;
    }
    // Constantes
    const float min_yaw = -Util::PI/2.0; //-90°
    const float max_yaw = Util::PI/2.0;  // 90°
    const float yaw_step = Util::PI/4.0; // 45°
    float yaw = min_yaw;
    ROS_INFO("PLACE: Mirando a primer punto de búsqueda");
    // Mirar al frente
    r_driver->head->lookAt(Util::BASE_FRAME, 1.5, 0, 0);
    // Contenedor de nube de puntos
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>()),
                              cloud_subsampled(new PointCloud<PointXYZ>()),
                              cloud_surface(new PointCloud<PointXYZ>());
    // Iterar y mirar alrededor
    while (yaw <= max_yaw){
        ROS_INFO("PLACE: Rotando cabeza a yaw = %f", Util::toGrad(yaw));
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
            ROS_INFO("PLACE: Refinación exitosa");
            // Superficie refinada. Guardar
            the_surface->setCloud(cloud_surface);
            the_surface->setNormal(surface_normal);
            the_surface->setCentroid(surface_centroid);
            // Agregar superficie al collision world
            PolygonMesh surface_triangles = Util::getTriangulation(cloud_surface);
            Util::addSurfaceAsCollisionObject(co_pub, surface_triangles);
            ROS_INFO("PLACE: Finaliza exitosamente adquisición con superficie de %d puntos", (int)cloud_surface->points.size());                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
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
    ROS_INFO("PLACE: Comienza posicionamiento de objeto");
    ros::NodeHandle nh_;
    ros::Publisher gripper_pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("gripper_pose",1);
    ros::Publisher gripper_future_pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("gripper_future_pose",1);

    // Eliminar puntos donde no puede ocurrir el placing
    // PointIndices::Ptr placing_points = Util::getFactiblePlacingPointsIndices((the_surface->cloud), the_surface->normal, the_object->base_area);

    // Volver a llevar gripper a pose de scan
    ROS_INFO("PLACE: Moviendo gripper activo a pose default");
    bool gotopose_ok;
    if (grasp_arm == 'l')
        gotopose_ok = r_driver->lgripper->goToPose(last_scan_pose);
    else
        gotopose_ok = r_driver->rgripper->goToPose(last_scan_pose);
    if (not gotopose_ok){
        ROS_ERROR("PLACE: No se pudo ir a la pose especificada");
        return false;
    }
    ROS_INFO("PLACE: Comienza búsqueda de pose para griper basada en estabilidad de objeto");
    // Obtener transformación, transformar pose estable y superficie a odom frame
    geometry_msgs::PoseStamped stable_pose;
    stable_pose.header.frame_id = Util::ODOM_FRAME;
    Eigen::Matrix4f gripper_to_odom_tf = Util::getTransformation(the_object->stable_pose.header.frame_id, Util::ODOM_FRAME);
    stable_pose.pose = Util::transformPose(the_object->stable_pose.pose, gripper_to_odom_tf); // OJO ACÁ, TRANSFORM DE POSE ESTABLE. OJO SI ESTO GENERA ERRORES
    geometry_msgs::PoseStamped gripper_current_pose = (grasp_arm == 'l' ? r_driver->lgripper->getCurrentPose() : r_driver->rgripper->getCurrentPose());
    stable_pose_pub.publish(stable_pose);
    stable_pose_pub.publish(stable_pose);
    stable_pose_pub.publish(stable_pose);
    gripper_pose_pub.publish(gripper_current_pose);
    gripper_pose_pub.publish(gripper_current_pose);
    gripper_pose_pub.publish(gripper_current_pose);
    // Calcular rotación de pose gripper, relativa a pose estable
    // Notación:    A: poses objeto
    //              B: poses gripper
    //              X: Incógnita
    tf::Quaternion qa (stable_pose.pose.orientation.x, stable_pose.pose.orientation.y, stable_pose.pose.orientation.z, stable_pose.pose.orientation.w);
    tf::Quaternion qb (gripper_current_pose.pose.orientation.x, gripper_current_pose.pose.orientation.y, gripper_current_pose.pose.orientation.z, gripper_current_pose.pose.orientation.w);
    tf::Quaternion qx = qa.inverse()*qb;
    // Para cálculos posteriores vectoriales
    Eigen::Vector3f va (stable_pose.pose.position.x, stable_pose.pose.position.y, stable_pose.pose.position.z);
    Eigen::Vector3f vb (gripper_current_pose.pose.position.x, gripper_current_pose.pose.position.y, gripper_current_pose.pose.position.z);
    Eigen::Vector3f vba = vb-va;                            // Este es el importante. Utilizado para obtener pose.position de gripper al final.
    geometry_msgs::Quaternion posb_q = Util::coefsToQuaternionMsg(vba.x(), vba.y(), vba.z());
    tf::Quaternion tf_posb_q (posb_q.x, posb_q.y, posb_q.z, posb_q.w);
    tf::Quaternion tf_a_posb = qa.inverse()*tf_posb_q;      // Este es el segundo importante. Con este se obtiene la pose.position del gripper haciendo qa*tf_a_posb

/*    // para testear que qx es correcto
    geometry_msgs::PoseStamped test_pose;
    test_pose.header.frame_id = Util::ODOM_FRAME;
    test_pose.pose.position = gripper_current_pose.pose.position;
    tf::Quaternion test_q = qa*qx;
    test_pose.pose.orientation.x = test_q.x();
    test_pose.pose.orientation.y = test_q.y();
    test_pose.pose.orientation.z = test_q.z();
    test_pose.pose.orientation.w = test_q.w();
    test_pose_pub.publish(test_pose);
    test_pose_pub.publish(test_pose);
    test_pose_pub.publish(test_pose);
    // */



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
        new_surface_pose.header.frame_id = Util::ODOM_FRAME;
        new_surface_pose.pose.position.x = (the_surface->cloud)->points[pc_indices[i]].x;
        new_surface_pose.pose.position.y = (the_surface->cloud)->points[pc_indices[i]].y;
        new_surface_pose.pose.position.z = (the_surface->cloud)->points[pc_indices[i]].z;
        new_surface_pose.pose.orientation = the_surface->normal.pose.orientation;
        surface_pose_pub.publish(new_surface_pose);
        surface_pose_pub.publish(new_surface_pose);
        surface_pose_pub.publish(new_surface_pose);
        tf::Quaternion new_surface_q (new_surface_pose.pose.orientation.x, new_surface_pose.pose.orientation.y, new_surface_pose.pose.orientation.z, new_surface_pose.pose.orientation.w);
        Eigen::Vector3f new_surface_position (new_surface_pose.pose.position.x, new_surface_pose.pose.position.y, new_surface_pose.pose.position.z);

        ROS_INFO("PLACE: Calculando y mostrando futura posible pose del gripper");
        geometry_msgs::PoseStamped future_gripper_pose;
        future_gripper_pose.header.frame_id = Util::ODOM_FRAME;
        // Cálculo de orientación: qgrippernueva = qnuevasuperficie*qx
        tf::Quaternion future_gripper_q = new_surface_q*qx;
        future_gripper_pose.pose.orientation.x = future_gripper_q.x();
        future_gripper_pose.pose.orientation.y = future_gripper_q.y();
        future_gripper_pose.pose.orientation.z = future_gripper_q.z();
        future_gripper_pose.pose.orientation.w = future_gripper_q.w();
        // Cálculo de posición:
        tf::Quaternion future_gripper_position_q = new_surface_q*tf_a_posb;
        geometry_msgs::Quaternion fgpq_temp;
        fgpq_temp.x = future_gripper_position_q.x();
        fgpq_temp.y = future_gripper_position_q.y();
        fgpq_temp.z = future_gripper_position_q.z();
        fgpq_temp.w = future_gripper_position_q.w();
        Eigen::Vector3f future_gripper_position = Util::quaternionMsgToVector(fgpq_temp)*vba.norm() + new_surface_position; // orientación y largo
        future_gripper_pose.pose.position.x = future_gripper_position.x();
        future_gripper_pose.pose.position.y = future_gripper_position.y();
        future_gripper_pose.pose.position.z = future_gripper_position.z();
        gripper_future_pose_pub.publish(future_gripper_pose);
        gripper_future_pose_pub.publish(future_gripper_pose);
        gripper_future_pose_pub.publish(future_gripper_pose);
        // Intentar ir a la pose
        ROS_INFO("PLACE: Intentando ir a pose...");
        if (grasp_arm == 'l')
            gotopose_ok = r_driver->lgripper->goToPose(future_gripper_pose);
        else
            gotopose_ok = r_driver->rgripper->goToPose(future_gripper_pose);
        if (gotopose_ok){
            ROS_INFO("PLACE: Pose alcanzada");
            // Soltar objeto
            // Quitar gripper
            // El gripper retrocederá una distancia determinada, en la misma dirección que su orientación
            geometry_msgs::PoseStamped gripper_backoff_pose = future_gripper_pose;
            // Eigen::Vector3f backoff_position = gripper_p - Util::PLACING_BACKOFF_DISTANCE*(gripper_q.normalized());
            Eigen::Vector3f backoff_position = Eigen::Vector3f(future_gripper_pose.pose.position.x, future_gripper_pose.pose.position.y, future_gripper_pose.pose.position.z) - Util::quaternionMsgToVector(future_gripper_pose.pose.orientation).normalized()*Util::PLACING_BACKOFF_DISTANCE;
            gripper_backoff_pose.pose.position.x = backoff_position.x();
            gripper_backoff_pose.pose.position.y = backoff_position.y();
            gripper_backoff_pose.pose.position.z = backoff_position.z();
            if (grasp_arm == 'l'){
                ROS_INFO("PLACE: Abriendo gripper izquierdo");
                r_driver->lgripper->setOpening(1, -1);
                Util::detachMeshFromGripper(aco_pub, co_pub);
                ros::Duration(1.0).sleep();
                ROS_INFO("PLACE: Retrocediendo gripper");
                r_driver->lgripper->goToPose(gripper_backoff_pose);
                ROS_INFO("PLACE: Cerrando gripper izquierdo");
                r_driver->lgripper->setOpeningNonBlocking(0, 1000);
            }
            else{
                ROS_INFO("PLACE: Abriendo gripper derecho");
                r_driver->rgripper->setOpening(1, -1);
                Util::detachMeshFromGripper(aco_pub, co_pub);
                ros::Duration(1.0).sleep();
                ROS_INFO("PLACE: Retrocediendo gripper");
                r_driver->rgripper->goToPose(gripper_backoff_pose);
                ROS_INFO("PLACE: Cerrando gripper derecho");
                r_driver->rgripper->setOpeningNonBlocking(0, 1000);
            }
            ROS_INFO("PLACE: Objeto exitosamente posicionado");
            return true;
        }
        // END TEST
        ros::Duration(0.3).sleep();
    }
    return false;
}
/*bool placeObject_old2(){
    ROS_INFO("PLACE: Comienza posicionamiento de objeto");
    ros::NodeHandle nh_;
    ros::Publisher gripper_pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("gripper_pose",1);
    ros::Publisher gripper_future_pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("gripper_future_pose",1);

    // Eliminar puntos donde no puede ocurrir el placing
    // PointIndices::Ptr placing_points = Util::getFactiblePlacingPointsIndices((the_surface->cloud), the_surface->normal, the_object->base_area);

    // Volver a llevar gripper a pose de scan
    ROS_INFO("PLACE: Moviendo gripper activo a pose default");
    bool gotopose_ok;
    if (grasp_arm == 'l')
        gotopose_ok = r_driver->lgripper->goToPose(last_scan_pose);
    else
        gotopose_ok = r_driver->rgripper->goToPose(last_scan_pose);
    if (not gotopose_ok){
        ROS_ERROR("PLACE: No se pudo ir a la pose especificada");
        return false;
    }
    ROS_INFO("PLACE: Comienza búsqueda de pose para griper basada en estabilidad de objeto");
    // Obtener transformación, transformar pose estable y superficie a baseframe
    geometry_msgs::PoseStamped stable_pose_odom;
    stable_pose_odom.header.frame_id = Util::ODOM_FRAME;
    Eigen::Matrix4f gripper_to_odom_tf = Util::getTransformation(the_object->stable_pose.header.frame_id, Util::ODOM_FRAME);
    stable_pose_odom.pose = Util::transformPose(the_object->stable_pose.pose, gripper_to_odom_tf); // OJO ACÁ, TRANSFORM DE POSE ESTABLE. OJO SI ESTO GENERA ERRORES
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
        new_surface_pose.header.frame_id = Util::ODOM_FRAME;
        new_surface_pose.pose.position.x = (the_surface->cloud)->points[pc_indices[i]].x;
        new_surface_pose.pose.position.y = (the_surface->cloud)->points[pc_indices[i]].y;
        new_surface_pose.pose.position.z = (the_surface->cloud)->points[pc_indices[i]].z;
        new_surface_pose.pose.orientation = the_surface->normal.pose.orientation;
        surface_pose_pub.publish(new_surface_pose);
        surface_pose_pub.publish(new_surface_pose);
        surface_pose_pub.publish(new_surface_pose);

        // Obtener transformación pose estable a pose alcanzable
        geometry_msgs::PoseStamped gripper_pose;
        if (grasp_arm == 'l')
            gripper_pose = r_driver->lgripper->getCurrentPose();
        else
            gripper_pose = r_driver->rgripper->getCurrentPose();
        ROS_INFO("PLACE: Transformando pose gripper a baseframe");
        gripper_pose_pub.publish(gripper_pose);
        gripper_pose_pub.publish(gripper_pose);
        gripper_pose_pub.publish(gripper_pose);
        ROS_INFO("PLACE: Calculando y mostrando futura posible pose del gripper");
        Eigen::Vector3f pose_ini_orientation = Util::quaternionMsgToVector(stable_pose_odom.pose.orientation);
        Eigen::Vector3f pose_end_orientation = Util::quaternionMsgToVector(new_surface_pose.pose.orientation);
        // ROTACION CON QUATERNION
        // Obtener quaternion de rotación entre pose inicial y final del objeto
        Eigen::Quaternionf rotation_q = Util::getQuaternionBetweenVectors(pose_ini_orientation, pose_end_orientation);
        // Rotar orientación y posición del gripper
        Eigen::Vector3f gripper_p (gripper_pose.pose.position.x, gripper_pose.pose.position.y, gripper_pose.pose.position.z);
        Eigen::Vector3f gripper_q = Util::quaternionMsgToVector(gripper_pose.pose.orientation);
        Eigen::Vector3f object_pose_ini (stable_pose_odom.pose.position.x, stable_pose_odom.pose.position.y, stable_pose_odom.pose.position.z);
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
        geometry_msgs::PoseStamped gripper_future_pose;
        gripper_future_pose.header.frame_id = Util::ODOM_FRAME;
        gripper_future_pose.pose.position.x = gripper_p[0];
        gripper_future_pose.pose.position.y = gripper_p[1];
        gripper_future_pose.pose.position.z = gripper_p[2];
        // Determinación de orientación final del gripper
        tf::Quaternion p1_q (stable_pose_odom.pose.orientation.x, stable_pose_odom.pose.orientation.y, stable_pose_odom.pose.orientation.z, stable_pose_odom.pose.orientation.w);
        tf::Quaternion p11_q (new_surface_pose.pose.orientation.x, new_surface_pose.pose.orientation.y, new_surface_pose.pose.orientation.z, new_surface_pose.pose.orientation.w);
        // tf::Quaternion factor_q = p11_q*p1_q.inverse(); // No funcionó
        tf::Quaternion factor_q = p1_q.inverse()*p11_q; // Tampoco funciona
        tf::Quaternion p2_q (gripper_pose.pose.orientation.x, gripper_pose.pose.orientation.y, gripper_pose.pose.orientation.z, gripper_pose.pose.orientation.w);
        tf::Quaternion p22_q = p2_q*factor_q;
        geometry_msgs::Quaternion new_gripper_q;
        new_gripper_q.x = p22_q.x();
        new_gripper_q.y = p22_q.y();
        new_gripper_q.z = p22_q.z();
        new_gripper_q.w = p22_q.w();
        gripper_future_pose.pose.orientation = new_gripper_q;
        // gripper_future_pose.pose.orientation = Util::coefsToQuaternionMsg(gripper_q[0], gripper_q[1], gripper_q[2]);
        // Publicar
        gripper_future_pose_pub.publish(gripper_future_pose);
        gripper_future_pose_pub.publish(gripper_future_pose);
        gripper_future_pose_pub.publish(gripper_future_pose);
        // Intentar ir a la pose
        ROS_INFO("PLACE: Intentando ir a pose...");
        if (grasp_arm == 'l')
            gotopose_ok = r_driver->lgripper->goToPose(gripper_future_pose);
        else
            gotopose_ok = r_driver->rgripper->goToPose(gripper_future_pose);
        if (gotopose_ok){
            ROS_INFO("PLACE: Pose alcanzada");
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
                Util::detachMeshFromGripper(aco_pub, co_pub);
                ros::Duration(1.0).sleep();
                ROS_INFO("PLACE: Retrocediendo gripper");
                r_driver->lgripper->goToPose(gripper_backoff_pose);
                ROS_INFO("PLACE: Cerrando gripper izquierdo");
                r_driver->lgripper->setOpeningNonBlocking(0, 1000);
            }
            else{
                ROS_INFO("PLACE: Abriendo gripper derecho");
                r_driver->rgripper->setOpening(1, -1);
                Util::detachMeshFromGripper(aco_pub, co_pub);
                ros::Duration(1.0).sleep();
                ROS_INFO("PLACE: Retrocediendo gripper");
                r_driver->rgripper->goToPose(gripper_backoff_pose);
                ROS_INFO("PLACE: Cerrando gripper derecho");
                r_driver->rgripper->setOpeningNonBlocking(0, 1000);
            }
            ROS_INFO("PLACE: Objeto exitosamente posicionado");
            return true;
        }
        // END TEST
        ros::Duration(0.3).sleep();
    }
    return false;
}*/

/*bool placeObject_old(){
    ROS_INFO("PLACE: Comienza posicionamiento de objeto");
    ros::NodeHandle nh_;
    ros::Publisher gripper_pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("gripper_pose",1);
    ros::Publisher gripper_future_pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("gripper_future_pose",1);

    // Eliminar puntos donde no puede ocurrir el placing
    // PointIndices::Ptr placing_points = Util::getFactiblePlacingPointsIndices((the_surface->cloud), the_surface->normal, the_object->base_area);

    // Volver a llevar gripper a pose de scan
    ROS_INFO("PLACE: Moviendo gripper activo a pose default");
    bool gotopose_ok;
    if (grasp_arm == 'l')
        gotopose_ok = r_driver->lgripper->goToPose(last_scan_pose);
    else
        gotopose_ok = r_driver->rgripper->goToPose(last_scan_pose);
    if (not gotopose_ok){
        ROS_ERROR("PLACE: No se pudo ir a la pose especificada");
        return false;
    }
    ROS_INFO("PLACE: Comienza búsqueda de pose para griper basada en estabilidad de objeto");
    // Obtener transformación, transformar pose estable y superficie a baseframe
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
        ROS_INFO("PLACE: Intentando ir a pose...");
        if (grasp_arm == 'l')
            gotopose_ok = r_driver->lgripper->goToPose(gripper_future_pose);
        else
            gotopose_ok = r_driver->rgripper->goToPose(gripper_future_pose);
        if (gotopose_ok){
            ROS_INFO("PLACE: Pose alcanzada");
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
                Util::detachMeshFromGripper(aco_pub, co_pub);
                ros::Duration(1.0).sleep();
                ROS_INFO("PLACE: Retrocediendo gripper");
                r_driver->lgripper->goToPose(gripper_backoff_pose);
                ROS_INFO("PLACE: Cerrando gripper izquierdo");
                r_driver->lgripper->setOpeningNonBlocking(0, 1000);
            }
            else{
                ROS_INFO("PLACE: Abriendo gripper derecho");
                r_driver->rgripper->setOpening(1, -1);
                Util::detachMeshFromGripper(aco_pub, co_pub);
                ros::Duration(1.0).sleep();
                ROS_INFO("PLACE: Retrocediendo gripper");
                r_driver->rgripper->goToPose(gripper_backoff_pose);
                ROS_INFO("PLACE: Cerrando gripper derecho");
                r_driver->rgripper->setOpeningNonBlocking(0, 1000);
            }
            ROS_INFO("PLACE: Objeto exitosamente posicionado");
            return true;
        }
        // END TEST

        ros::Duration(0.3).sleep();
    }
    return false;
    
}*/
void place(geometry_msgs::PoseStamped placing_pose){
    vector<moveit_msgs::PlaceLocation> loc;
    moveit_msgs::PlaceLocation g;
    g.place_pose = placing_pose;

    g.pre_place_approach.direction.vector.z = -1.0;
    g.post_place_retreat.direction.vector.x = -1.0;
    g.post_place_retreat.direction.header.frame_id = Util::BASE_FRAME;
    string gripper_frame_id = (grasp_arm == 'l' ? "l" : "r") + Util::GRIPPER_LINK_PREFIX;
    g.pre_place_approach.direction.header.frame_id = gripper_frame_id;
    // stringstream gripper_frame_id;
    // gripper_frame_id << grasp_arm << "_wrist_roll_link";
    // g.pre_place_approach.direction.header.frame_id = gripper_frame_id.str();
    g.pre_place_approach.min_distance = 0.1;
    g.pre_place_approach.desired_distance = 0.2;
    g.post_place_retreat.min_distance = 0.1;
    g.post_place_retreat.desired_distance = 0.25;
    string gripper_joint = (grasp_arm == 'l' ? "l" : "r") + Util::GRIPPER_JOINT_PREFIX;
    // stringstream gripper_joint;
    // gripper_joint << grasp_arm << "_gripper_joint";
    g.post_place_posture.joint_names.resize(1, gripper_joint);
    g.post_place_posture.points.resize(1);
    g.post_place_posture.points[0].positions.resize(1);
    g.post_place_posture.points[0].positions[0] = 1;
    ROS_INFO("PLACE: Intentando posicionar con place de moveit");
    loc.push_back(g);
    if (grasp_arm == 'l'){
        r_driver->lgripper->moveit_group_->setSupportSurfaceName(Util::COLLISION_SURFACE_ID);
        // r_driver->lgripper->moveit_group_->setPlannerId("RRTConnectkConfigDefault");
        r_driver->lgripper->moveit_group_->place(Util::COLLISION_MESH_ID, loc);
    }
    else{
        r_driver->rgripper->moveit_group_->setSupportSurfaceName(Util::COLLISION_SURFACE_ID);
        // r_driver->rgripper->moveit_group_->setPlannerId("RRTConnectkConfigDefault");
        r_driver->rgripper->moveit_group_->place(Util::COLLISION_MESH_ID, loc);   
    }
    ROS_INFO("PLACE: Fin place moveit");
}

/**
 * main: Ejecuta todo lo necesario para efectuar placing
 * @param argc: Cuenta de argumentos
 * @param argv: Recibe como argumento la mano donde está el objeto. Opcionalmente, recibe radio máximo de volumen de objeto. Default es Util::COLLISION_BALL_RADIUS.
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
    if (argc >= 3){
        Util::COLLISION_BALL_RADIUS = atof(argv[2]);
        ROS_INFO("Usando objeto de radio NO MAYOR a %fm", Util::COLLISION_BALL_RADIUS);
    }
    else{
        ROS_INFO("Usando valor default para radio de objeto: %fm", Util::COLLISION_BALL_RADIUS);    
    }
    // Iniciar 1nodo ROS
    ros::init(argc, argv, "placing_node");
    ros::NodeHandle nh;
    // Para terminar con ctrl+c
    signal(SIGINT, signalHandler);
    // Iniciando robot driver
    ROS_INFO("PLACE: Iniciando RobotDriver");
    r_driver = new RobotDriver();
    

    // vector<string> joints;
    // if (grasp_arm == 'l')
    //     joints = r_driver->lgripper->moveit_group_->getJoints();
    // else
    //     joints = r_driver->rgripper->moveit_group_->getJoints();
    // for (int  i=0; i<(int)joints.size(); i++){
    //     printf("Joint[%d]: %s\n", i, joints[i].c_str());
    // }






    
    aco_pub = nh.advertise<moveit_msgs::AttachedCollisionObject>("/attached_collision_object", 10);
    co_pub = nh.advertise<moveit_msgs::CollisionObject>("/collision_object", 10);
    Util::enableDefaultGripperCollisions(aco_pub, co_pub, true, grasp_arm);
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
    test_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("test_pose", 1);

    

    ros::Duration(1).sleep();

// TESTEANDO NUEVO ORDEN: SCAN -> POSE ESTABLE -> SEARCH SURFACE -> PLACE


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
    stable_pose_pub.publish(the_object->stable_pose);
    stable_pose_pub.publish(the_object->stable_pose);
    stable_pose_pub.publish(the_object->stable_pose);
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
        [DONE]- Testear y reparar obtención de pose de placing
        [NOT]- Revisar problema de head driver de no hacer nada a veces (lanzar timeout)
        [DONE]- Refinar filtro del gripper scanner:
            1) Filtrar mejor el entorno
                Usar clustering?
            2) Reintentar poses si no funcionaron
        [DONE]- Reparar dirección de pose encontrada
            Siempre debe apuntar hacia "adentro"
        - Agregar vista desde "arriba" al scaneo
        [NOT]- Al llegar a la superficie, el robot debiera girar para quedar de frente al centroide de la superficie
    opcionales:
        [DONE]- Evaluar mover método "getStablePose" de Util a este archivo.
        - Evaluar corregir vista de la superficie encontrada (una vez encontrada, mirar hacia ella, volver a buscar y corregir nube)
        - Evaluar reparar head driver para rotar pitch
        - Evaluar eliminar pose inicial del head driver al inicializarlo
        - Evaluar suavizar (o "promediar") nube merged del objeto (separado del gripper)
        - Evaluar mirar al tool en vez del inicio del gripper en el escaneo
        - Evaluar habilitar movimientos simultáneos (cabeza + gripper por ejemplo, 2 grippers...)
 */