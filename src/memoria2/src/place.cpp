/*
 Este es el nodo principal de la memoria.
 Este programa se preocupa de hacer todo lo necesario para lograr que el robot posicione el objeto
 */
#include <string>
#include <vector>
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
#include "Polymesh/Polymesh.h"


using namespace std;
using namespace pcl;

// CONSTANTES

// VARIABLES GLOBALES
RobotDriver *r_driver;
char grasp_arm;

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
bool searchSurface(PointCloud<PointXYZ>::Ptr &cloud_out, geometry_msgs::PoseStamped &surface_normal){
    bool gotopose_ok;
    // Mover gripper no activo a posición donde no moleste
    ROS_DEBUG("PLACE: Quitar del campo visual al gripper inactivo");
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
    // Mover gripper con objeto a posición donde no moleste
    ROS_DEBUG("PLACE: Quitar del campo visual al gripper con objeto");
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
    ModelCoefficients::Ptr cloud_coefs(new ModelCoefficients());
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
        if (Util::searchPlacingSurface(cloud_subsampled, cloud_surface, surface_normal, 0.3, 0.8, Util::DEFAULT_DESIRED_PITCH)){
            ROS_DEBUG("PLACE: Encontrada");

            ROS_DEBUG("PLACE: Refinando superficie");
            // Repasar superficie:
            //      1) Obtener centroide
            //      2) Mirar centroide
            //      3) Buscar superficie denuevo
            //      4) Iterar N veces
            for (int i=0; i<Util::SURFACE_REFINING_ITERATIONS; i++){
                ROS_DEBUG("PLACE: Iteracion %d de %d", i, Util::SURFACE_REFINING_ITERATIONS);
                // Obtener centroide
                geometry_msgs::Point centroid = Util::getCloudCentroid(cloud_surface);
                // Mirar centroide
                r_driver->head->lookAt(cloud_surface->header.frame_id, centroid.x, centroid.y, centroid.z);
                ros::Duration(Util::KINECT_STABILIZE_TIME).sleep();
                // Buscar superficie denuevo
                cloud = r_driver->sensors->kinect->getNewCloud();
                cloud_subsampled = Util::subsampleCloud(cloud, Util::SUBSAMPLE_LEAFSIZE);
                if (not Util::searchPlacingSurface(cloud_subsampled, cloud_surface, surface_normal, centroid.z - Util::SURFACE_REFINING_THRESHOLD, centroid.z + Util::SURFACE_REFINING_THRESHOLD, Util::DEFAULT_DESIRED_PITCH)){
                    ROS_ERROR("PLACE: Superficie no pudo ser refinada. Este error no debiera ocurrir. Abortando");
                    return false;
                }
            }
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
        if (Util::searchPlacingSurface(cloud_subsampled, cloud_surface, surface_normal, 0.3, 0.8, Util::DEFAULT_DESIRED_PITCH)){
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
        closest_pose_pub.publish(closest_pose);
        ROS_DEBUG("PLACE: Yendo a la pose...");
        if (not r_driver->base->goToPose(closest_pose)){
            ROS_ERROR("PLACE: No se pudo ir a la pose");
            return false;
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
    return true;    
}
bool scanGripper(PointCloud<PointXYZ>::Ptr &object_out, PointCloud<PointXYZ>::Ptr &gripper_out){
    bool gotopose_ok;
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
        ROS_DEBUG("PLACE: nube recibida desde kinect: %d puntos, frame: %s", (int)cloud->points.size(), cloud->header.frame_id.c_str());
        ROS_DEBUG("PLACe: Primer punto está en (%f, %f, %f)", cloud->points[0].x, cloud->points[1].y, cloud->points[2].z);
        
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
            ROS_DEBUG("PLACE: Se obtuvieron todos los scans. Uniendo...");
            PointCloud<PointXYZ>::Ptr merged(new PointCloud<PointXYZ>());
            for (int i=0; i<cloud_scans.size(); i++){
                *merged += cloud_scans[i];
            }
            merged->header.frame_id = GRIPPER_FRAME;
            if (merged->points.size() == 0){
                ROS_ERROR("PLACE: El scanneo del gripper retornó 0 puntos!");
                return false;
            }
            ROS_DEBUG("PLACE: Union resulto en nube de %d puntos", (int)merged->points.size());
            // Filtrar gripper_out
            ROS_DEBUG("PLACE: Filtrando scans");
            if (not Util::gripperFilter(merged, object_out, gripper_out)){
                ROS_ERROR("PLACE: Algo ocurrio al intentar filtrar el gripper");
                return false;
            }
            ROS_DEBUG("PLACE: Objeto: %d puntos. Gripper: %d puntos.", (int)object_out->points.size(), (int)gripper_out->points.size());
            object_out->header.frame_id = GRIPPER_FRAME;
            gripper_out->header.frame_id = GRIPPER_FRAME;
            return true;
        }
        ROS_DEBUG("PLACE: Posicionandose psra siguiente scan");
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
 * getStableObjectPose Se encarga de calcular la mejor pose para el objeto. El resultado
 *                 es una pose, relativa al gripper, que indica la posición más estable para el objeto.
 * @param  object_pc  Nube de puntos del objeto
 * @param  gripper_pc nube de puntos del gripper
 * @param  pose_out   Pose resultante, RELATIVA AL GRIPPER
 * @return            true en éxito, false en caso contrario
 */
bool getStableObjectPose(PointCloud<PointXYZ>::Ptr object_pc, PointCloud<PointXYZ>::Ptr gripper_pc, geometry_msgs::PoseStamped &pose_out){
    /*    // Obtener mejor superficie
    if (not Util::getStablePose(object_pc, gripper_pc, pose_out)){
        ROS_ERROR("PLACE: Algo ocurrió al intentar obtener la pose estable");
        return false;
    }
    return true;*/
        // Verificar que ambas nubes están en el mismo frame
    if (object_pc->header.frame_id.compare(gripper_pc->header.frame_id) != 0){
        ROS_ERROR("UTIL: Nube de objeto y de gripper deben estar en el mismo frame de referencia");
        return false;
    }
    // Crear convex hull del objeto. Internamente Polymesh calcula varias cosas útiles
    Polymesh mesh = Polymesh(Util::getConvexHull(object_pc));
    // Obtener listado de posibles parches estables
    vector<vector<int> > patches; // Todos los parches posibles, ordenados desde el más grande al más pequeño
    vector<double> patches_areas; // Sus areas correspondientes, en el mismo orden anterior.
    mesh.getFlatPatches(Util::PATCH_ANGLE_THRESHOLD, patches, patches_areas);
    // Obtener centro de masa
    PointXYZ cm = mesh.getCenterOfMass();
    // Iterar hasta encontrar un parche estable. Priorizar parches grandes
    vector<int> best_patch;
    double best_patch_area;
    PointCloud<PointXYZ>::Ptr patch_plane(new PointCloud<PointXYZ>());
    ModelCoefficients::Ptr patch_plane_coefs(new ModelCoefficients());
    PointXYZ cm_proj;
    for (int i=0; i<patches.size(); i++){
        // Obtener plano representado por el parche
        mesh.flattenPatch(patches[i], *patch_plane, patch_plane_coefs);
        // proyectar centro de masa sobre plano
        cm_proj = Polymesh::projectPointOverFlatPointCloud(cm, patch_plane);
        // VERIFICACIÓN DE CONDICIONES:
        //      - Es un plano estable?
        //          * centro de masa se proyecta sobre parche?
        //      - Puede el gripper llegar a esa posición?
        //          * El gripper no es cortado por el plano de la superficie?
        if (Polymesh::isPointInConvexPolygon(cm_proj, *patch_plane) and not Util::isPointCloudCutByPlane(gripper_pc, patch_plane_coefs, patch_plane->points[0])){
            best_patch = patches[i];
            best_patch_area = patches_areas[i];
            ROS_INFO("PLACE: Se ha encontrado un plano estable (de area %.2f)\n", best_patch_area);
            // Ajustar dirección de la normal de la pose (debe apuntar hacia dentro del gripper)
            float alpha = Util::angleBetweenVectors(cm_proj.x, cm_proj.y, cm_proj.z, patch_plane_coefs->values[0], patch_plane_coefs->values[1], patch_plane_coefs->values[2]);
            ROS_DEBUG("PLACE: pose %s será invertida", alpha < Util::PI/2.0 ? "SI" : "NO");
            int invert = (alpha < Util::PI/2.0 ? -1 : 1);
            // guardar pose
            pose_out.pose.position.x = cm_proj.x;
            pose_out.pose.position.y = cm_proj.y;
            pose_out.pose.position.z = cm_proj.z;
            pose_out.pose.orientation = Util::coefsToQuaternionMsg(patch_plane_coefs->values[0]*invert, patch_plane_coefs->values[1]*invert, patch_plane_coefs->values[2]*invert);
            pose_out.header.frame_id = object_pc->header.frame_id; // frame del gripper "tool"
            ROS_INFO("PLACE: Pose guardada, terminando");
            return true;
        }
    }
    return false;
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
bool placeObject(geometry_msgs::PoseStamped stable_pose, PointCloud<PointXYZ>::Ptr surface_pc, geometry_msgs::PoseStamped surface_normal_pose){
    ros::NodeHandle nh_;
    ros::Publisher gripper_pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("gripper_pose",1);
    ros::Publisher gripper_future_pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("gripper_future_pose",1);

    // test de correctitud de transformación 
    ros::Publisher test_pose_ini_pub = nh_.advertise<geometry_msgs::PoseStamped>("pose_ini_unitv", 1);
    ros::Publisher test_pose_end_pub = nh_.advertise<geometry_msgs::PoseStamped>("pose_end_unitv", 1);
    ros::Publisher test_pose_transformed_end_pub = nh_.advertise<geometry_msgs::PoseStamped>("pose_end_transformed", 1);
    ros::Publisher test_pose_transformed_end_quat_pub = nh_.advertise<geometry_msgs::PoseStamped>("pose_end_transformed_quat", 1);


    // Obtener transformación, transformar pose y plano a baseframe
    geometry_msgs::PoseStamped stable_pose_base;
    stable_pose_base.header.frame_id = Util::BASE_FRAME;
    Eigen::Matrix4f gripper_to_base_tf = Util::getTransformation(stable_pose.header.frame_id, Util::BASE_FRAME);
    stable_pose_base.pose = Util::transformPose(stable_pose.pose, gripper_to_base_tf);
    Eigen::Matrix4f odom_to_base_tf = Util::getTransformation(Util::ODOM_FRAME, Util::BASE_FRAME);
    PointCloud<PointXYZ>::Ptr surface_base_pc (new PointCloud<PointXYZ>());
    transformPointCloud(*surface_pc, *surface_base_pc, odom_to_base_tf);
    // Iterar (hasta éxito o hasta máximo de intentos)
    for (int i=0; i<(int)surface_pc->points.size(); i++){
        ROS_DEBUG("PLACE: Mostrando posible punto de placing (indice %d)", i);
        // Construyo una posible pose, basado en puntos de la superficie y la normal transformada
        geometry_msgs::PoseStamped new_surface_pose;
        new_surface_pose.header.frame_id = Util::BASE_FRAME;
        new_surface_pose.pose.position.x = surface_pc->points[i].x;
        new_surface_pose.pose.position.y = surface_pc->points[i].y;
        new_surface_pose.pose.position.z = surface_pc->points[i].z;
        new_surface_pose.pose.orientation = (Util::transformPose(surface_normal_pose.pose, odom_to_base_tf)).orientation;
        surface_pose_pub.publish(new_surface_pose);
        surface_pose_pub.publish(new_surface_pose);
        surface_pose_pub.publish(new_surface_pose);
        // obtener transformación pose estable a pose alcanzable
        // Eigen::Matrix4f gripper_to_surface_tf = Util::getTransformBetweenPoses(stable_pose_base.pose, new_surface_pose.pose);
        
        geometry_msgs::PoseStamped gripper_pose;
        if (grasp_arm == 'l')
            gripper_pose = r_driver->lgripper->getCurrentPose();
        else
            gripper_pose = r_driver->rgripper->getCurrentPose();
        ROS_DEBUG("PLACE: Transformando pose gripper a baseframe");
        geometry_msgs::PoseStamped gripper_pose_base;
        gripper_pose_base.pose = Util::transformPose(gripper_pose.pose, Util::getTransformation(gripper_pose.header.frame_id, Util::BASE_FRAME));
        gripper_pose_base.header.frame_id = Util::BASE_FRAME;
        gripper_pose_pub.publish(gripper_pose_base);
        gripper_pose_pub.publish(gripper_pose_base);
        gripper_pose_pub.publish(gripper_pose_base);
        ROS_DEBUG("PLACE: Calculando y mostrando futura posible pose del gripper");
        geometry_msgs::PoseStamped gripper_future_pose;
        gripper_future_pose.header.frame_id = Util::BASE_FRAME;
        Eigen::Vector3f pose_ini_orientation = Util::quaternionMsgToVector(stable_pose_base.pose.orientation);
        Eigen::Vector3f pose_end_orientation = Util::quaternionMsgToVector(new_surface_pose.pose.orientation);
        // Calcular matriz rotación R = I + [v]x + [v]x^2(1-c/s)
        // Recordar: Esto manda: stable_pose_base -> new_surface_pose
        //           se quiere llevar gripper_pose_base a una pose correcta
        // Calcular rotación
        Eigen::Matrix3f R = Util::getRotationBetweenVectors(pose_ini_orientation, pose_end_orientation);
        // TESTEO DE R
        // Ahora, pruebo si efectivamente R me lleva desde pose inicial a pose final
        Eigen::Vector3f test_orientation_obj_end = R*pose_ini_orientation;
        geometry_msgs::PoseStamped test_pose_transformed_end;
        test_pose_transformed_end.header.frame_id = Util::BASE_FRAME;
        test_pose_transformed_end.pose.position = new_surface_pose.pose.position;
        test_pose_transformed_end.pose.orientation = Util::coefsToQuaternionMsg(test_orientation_obj_end[0], test_orientation_obj_end[1], test_orientation_obj_end[2]);
        test_pose_transformed_end_pub.publish(test_pose_transformed_end);
        test_pose_transformed_end_pub.publish(test_pose_transformed_end);
        test_pose_transformed_end_pub.publish(test_pose_transformed_end);

        // ROTACION CON QUATERNION
        Eigen::Vector3f a = pose_ini_orientation.cross(pose_end_orientation);
        Eigen::Quaternionf rotation_q, p, rotated_p;
        rotation_q.x() = a.x();
        rotation_q.y() = a.y();
        rotation_q.z() = a.z();
        rotation_q.w() = 1 + pose_ini_orientation.dot(pose_end_orientation);
        rotation_q.normalize();

        p.w() = 0;
        p.vec() = pose_ini_orientation;
        rotated_p = rotation_q*p*rotation_q.inverse();
        rotated_p.normalize();
        Eigen::Vector3f rotated_orientation = rotated_p.vec();
        // TEST ROTACION CON QUATERNION
        geometry_msgs::PoseStamped test_pose_transformed_end_quat;
        test_pose_transformed_end_quat.header.frame_id = Util::BASE_FRAME;
        test_pose_transformed_end_quat.pose.position = new_surface_pose.pose.position;
        test_pose_transformed_end_quat.pose.orientation = Util::coefsToQuaternionMsg(rotated_orientation[0], rotated_orientation[1], rotated_orientation[2]);
        test_pose_transformed_end_quat_pub.publish(test_pose_transformed_end_quat);
        test_pose_transformed_end_quat_pub.publish(test_pose_transformed_end_quat);
        test_pose_transformed_end_quat_pub.publish(test_pose_transformed_end_quat);

        // Usar valores de pose que manda para transformar gripper_pose_base
        Eigen::Vector3f gripper_p (gripper_pose_base.pose.position.x, gripper_pose_base.pose.position.y, gripper_pose_base.pose.position.z);
        Eigen::Vector3f gripper_q = Util::quaternionMsgToVector(gripper_pose_base.pose.orientation);
        Eigen::Vector3f p1 (stable_pose_base.pose.position.x, stable_pose_base.pose.position.y, stable_pose_base.pose.position.z);
        Eigen::Vector3f p2 (new_surface_pose.pose.position.x, new_surface_pose.pose.position.y, new_surface_pose.pose.position.z);
        gripper_p -= p1;
        gripper_p = R*gripper_p;
        gripper_q = R*gripper_q;
        gripper_p += p2;
        gripper_p[2] += Util::PLACING_Z_MARGIN;
        // calcula pose ideal para el gripper + delta en Z
        /*  gripper_future_pose.pose.position.x = gripper_p[0];
        gripper_future_pose.pose.position.y = gripper_p[1];
        gripper_future_pose.pose.position.z = gripper_p[2];
        gripper_future_pose.pose.orientation = Util::coefsToQuaternionMsg(gripper_q[0], gripper_q[1], gripper_q[2]);
        */
        // Rotar pose alrededor de la normal de la pose tentativa
        if (gripper_q[0] < 0){
            ROS_DEBUG("PLACE: Pose apunta hacia el robot... rotando");
            Eigen::AngleAxis<float> rotate(Util::PI, Util::quaternionMsgToVector(new_surface_pose.pose.orientation));
            gripper_p -= p2;
            gripper_p = rotate*gripper_p;
            gripper_p += p2;
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
        /*bool gotopose_ok;
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
                r_driver->lgripper->setOpening(1, -1);
                r_driver->lgripper->goToPose(gripper_backoff_pose);
                r_driver->lgripper->setOpening(0, 1);
            }
            else{
                r_driver->rgripper->setOpening(1, -1);
                r_driver->rgripper->goToPose(gripper_backoff_pose);
                r_driver->rgripper->setOpening(0, 1);
            }
            return true;
        }*/
        // END TEST

        ros::Duration(0.3).sleep();
    }
    return false;
    
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

// TESTEANDO GOTOPOSE DE BASE
/*    // Mover 1m hacia atrás y mirar al frente
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = Util::BASE_FRAME;
    pose.pose.position.x = 2;
    pose.pose.position.y = pose.pose.position.z = 0;
    pose.pose.orientation = Util::coefsToQuaternionMsg(1,1,0);
    r_driver->base->goToPose(pose);*/

// TESTEANDO SETOPENING DE GRIPPERS
/*    r_driver->lgripper->setOpening(1, 400);
    r_driver->rgripper->setOpening(1, 400);
    r_driver->lgripper->setOpening(0, 400);
    r_driver->rgripper->setOpening(0, 400);*/

// TESTEANDO GETNEWCLOUD DE KINECT
/*    cloud_pub = nh.advertise<PointCloud<PointXYZ> >("nube_maravillosa", 1);
    PointCloud<PointXYZ>::Ptr kinect_cloud (new PointCloud<PointXYZ>());
    ROS_DEBUG("PLACE: creado topico e iniciando ciclo");
    while (1){
        kinect_cloud = r_driver->sensors->kinect->getNewCloud();
        ROS_DEBUG("PLACE: N° Puntos: %d, frame: %s", (int)kinect_cloud->points.size(), kinect_cloud->header.frame_id.c_str());
        cloud_pub.publish(*kinect_cloud);
        ros::Duration(0.4).sleep();    
    }
*/
// TESTEANDO GOTOPOSE DE GRIPPERS
/*    ROS_DEBUG("PLACE: Llevando gripper izquierdo al frente");
    geometry_msgs::PoseStamped gripper_pose;
    gripper_pose.header.frame_id = Util::BASE_FRAME;
    gripper_pose.pose.position.x = 0.7;
    gripper_pose.pose.position.y = 0.3;
    gripper_pose.pose.position.z = 1.2;
    r_driver->lgripper->goToPose(gripper_pose);
    ROS_DEBUG("PLACE: Llevando gripper derecho al frente");
    gripper_pose.pose.position.y = -0.3;
    r_driver->rgripper->goToPose(gripper_pose);*/

// TESTEANDO SCANGRIPPER
/*    cloud_pub = nh.advertise<PointCloud<PointXYZ> >("object_pc", 1);
    cloud_pub2 = nh.advertise<PointCloud<PointXYZ> >("gripper_pc", 1);
    ros::Duration(1).sleep();
    PointCloud<PointXYZ>::Ptr object_pc (new PointCloud<PointXYZ>()),
                              gripper_pc (new PointCloud<PointXYZ>());
    ROS_DEBUG("PLACE: Escaneando gripper...");
    if (not scanGripper(object_pc, gripper_pc)){
        ROS_ERROR("No se pudo escanear gripper");
        endProgram(1);
    }
    ROS_DEBUG("PLACE: Publicando ambas nubes...");
    cloud_pub.publish(*object_pc);
    cloud_pub2.publish(*gripper_pc);*/


/*
    cloud_pub2 = nh.advertise<PointCloud<PointXYZ> >("punto_trans_pc", 1);
    point_pub = nh.advertise<geometry_msgs::PointStamped>("punto_mas_cercano", 1);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose_mas_cercana", 1);
    
    ros::Duration(1).sleep();*/
    
/*    geometry_msgs::PoseStamped placing_pose;
    if (not getStableObjectPose(placing_pose)){
        ROS_ERROR("PLACE: No se pudo obtener una pose de placing");
        endProgram(1);
    }*/
    
// TESTEANDO SEARCHSURFACE + GOTOPOSE + SCANGRIPPER + GETSTABLEOBJECTPOSE
    
    surface_pc_pub = nh.advertise<PointCloud<PointXYZ> >("surface", 1);
    object_pc_pub = nh.advertise<PointCloud<PointXYZ> >("object_pc", 1);
    gripper_pc_pub = nh.advertise<PointCloud<PointXYZ> >("gripper_pc", 1);

    closest_point_pub = nh.advertise<geometry_msgs::PointStamped>("closest_point", 1);
    // surface_centroid_pub = nh.advertise<geometry_msgs::PointStamped>("surface_centroid", 1);
    closest_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("closest_pose", 1);
    stable_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("stable_pose", 1);
    surface_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("surface_pose", 1);

    ros::Duration(1).sleep();
    PointCloud<PointXYZ>::Ptr surface_cloud (new PointCloud<PointXYZ>());
    geometry_msgs::PoseStamped surface_normal_pose;
    if (not searchSurface(surface_cloud, surface_normal_pose)){
        ROS_ERROR("No se pudo obtener superficie");
        endProgram(1);
    }
    ROS_INFO("PLACE: Superficie encontrada (frame: %s). Publicando...", surface_cloud->header.frame_id.c_str());
    surface_pc_pub.publish(*surface_cloud);
    /*    ROS_INFO("PLACE: Dirigiéndose hacia ella");
    if (not moveToSurface(surface_cloud)){
        ROS_ERROR("No se pudo ir hacia la superficie");
        endProgram(1);
    }*/
    PointCloud<PointXYZ>::Ptr object_pc (new PointCloud<PointXYZ>()),
                              gripper_pc (new PointCloud<PointXYZ>());
    ROS_DEBUG("PLACE: Escaneando gripper...");
    if (not scanGripper(object_pc, gripper_pc)){
        ROS_ERROR("No se pudo escanear gripper");
        endProgram(1);
    }
    ROS_DEBUG("PLACE: Publicando ambas nubes...");
    object_pc_pub.publish(*object_pc);
    gripper_pc_pub.publish(*gripper_pc);
    ROS_DEBUG("PLACE: Buscando pose de placing");
    geometry_msgs::PoseStamped placing_pose;
    if (not getStableObjectPose(object_pc, gripper_pc, placing_pose)){
        ROS_ERROR("No se pudo obtener pose de placing");
        endProgram(1);
    }
    ROS_DEBUG("PLACE: Publicando pose de placing...");
    stable_pose_pub.publish(placing_pose);
    stable_pose_pub.publish(placing_pose);
    stable_pose_pub.publish(placing_pose);
    if (not placeObject(placing_pose, surface_cloud, surface_normal_pose)){   
        ROS_ERROR("No se pudo hacer placing");
        endProgram(1);
    }
    ROS_INFO("PLACE: Objeto posicionado con exito!!!");
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