/*
La labor de este programa es escanear el objeto que se tenga en el gripper para luego buscar su mejor pose de placing.

Algoritmo:
    - Llevar gripper a posición de escaneo
    - Tomar imágenes de distintas perspectivas
    - Mezclar correctamente nubes de puntos
    - Filtrar lo mejor posible el gripper
*/
#include <string>
//#include <sstream>
#include <csignal>
#include <ctime>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <memoria/LookAt.h>
#include <memoria/LookAtMsg.h>
#include <memoria/ErrorMsg.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf/transform_listener.h>

using namespace std;
using namespace pcl;

// CONSTANTES
const double PI = 3.1416;
const double passthrough_z = 0.5;
const double LEAFSIZE = 0.005;
// Pose de escaneo
double scan_pose1_position[] = {0.5, 0, 1.5}; // X, Y, Z
double scan_pose1_orientation[] = {0, -PI/2, 0}; // R, P, Y 
geometry_msgs::PoseStamped scan_pose1;

const double roll_delta = PI/4;
const string KINECT_TOPIC = "head_mount_kinect/depth/points";
const string GRIPPER_FRAME_SUFFIX = "_wrist_roll_link";
const double WAIT_TF_TIMEOUT = 1.0;
const int TF_RETRYS = 5;

// VARIABLES GLOBALES
moveit::planning_interface::MoveGroup::Plan planner;
moveit::planning_interface::MoveGroup *gripper_group;
ros::ServiceClient lookat_client;
memoria::LookAt lookat_srv;
char GROUP = 'l'; // Puede ser 'l' o 'r'
bool listen_kinect = false;
vector<PointCloud<PointXYZ> > scans;

void signalHandler( int signum ){
    ros::shutdown();
    exit (EXIT_SUCCESS);
}
/*geometry_msgs::Pose newPose(float x, float y, float z, float ox, float oy, float oz, float w){
    geometry_msgs::Pose newpose;
    newpose.position.x  = x;
    newpose.position.y  = y;
    newpose.position.z  = z;
    newpose.orientation.x = ox;
    newpose.orientation.y = oy;
    newpose.orientation.z = oz;
    newpose.orientation.w  = w;
    return newpose;
}*/
void moveToPose(const geometry_msgs::PoseStamped &pose){
    gripper_group->setPoseTarget(pose);
    ROS_INFO("Planeando para pose (%.2f, %.2f, %.2f) - (%.2f, %.2f, %.2f, %.2f)", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);

    bool planDone = gripper_group->plan(planner);
    ROS_INFO("Plan %s", planDone ? "EXITOSO" : "FALLIDO");

    if (planDone)
    {
        ROS_INFO("Ejecutando plan...");
        gripper_group->move();
    }
}
bool lookAt(string frame_id, double x, double y, double z, bool rotate = false){
    /* 
    Recibe: Frame_id
            Punto en el espacio, si rotate == false
            Yaw, pitch, whatever, si rotate == true
    */
    memoria::LookAtMsg lookatmsg;
    lookatmsg.frame_id = frame_id;
    lookatmsg.rotate = rotate ? 1 : 0;
    lookatmsg.vector.x = x;
    lookatmsg.vector.y = y;
    lookatmsg.vector.z = z;
    lookat_srv.request.lookatmsg = lookatmsg;
    if (lookat_client.call(lookat_srv)){
        //ROS_INFO("Cabeza movida");
        return true;
    }
    else{
        ROS_ERROR("Error al llamar al servicio 'look_at'");
        return false;
    }
}
PointCloud<PointXYZ> mergeViews(){
    /* Recorre arreglo de pointclouds y retorna sólo una, mezclada */
    PointCloud<PointXYZ> merged;
    merged.height = 1;
    // Agregar primer elemento
    // Obtener transformación desde el frame de la kinect al del gripper
   
    /*// Convertir nube de puntos a mensaje ROS
    PCLPointCloud2 scan_pclmsg;
    toPCLPointCloud2(scans[0], scan_pclmsg);
    sensor_msgs::PointCloud2 scan_msg;
    pcl_conversions::fromPCL(scan_pclmsg, scan_msg);
    // Aplicar transformación a primer elemento
    sensor_msgs::PointCloud2 scan_transformed;
    pcl_ros::transformPointCloud(wrist_frame.str(), scan_msg, scan_transformed);
    // Volver a convertir a mensaje PCL
    pcl_conversions::toPCL(scan_transformed, scan_pclmsg);
    fromPCLPointCloud2(scan_pclmsg, merged);
    PointCloud<PointXYZ> temp;
    for (int i=1; i<scans.size(); i++){
        // Aplicar transformación a cada nube, y rotar según el ángulo roll_delta
        toPCLPointCloud2(scans[i], scan_pclmsg);
        pcl_conversions::fromPCL(scan_pclmsg, scan_msg);
        tf_listener.transformPointCloud(wrist_frame.str(), scan_msg, scan_transformed);
        pcl_conversions::toPCL(scan_transformed, scan_pclmsg);
        fromPCLPointCloud2(scan_pclmsg, temp);
        merged += temp;
    }
    return merged;*/
    //pcl_ros::transformPointCloud(wrist_frame.str(), scans[0], merged, tf_listener);
    //PointCloud<PointXYZ> temp;
    merged = scans[0];
    for (int i=1; i<scans.size(); i++){
        //pcl_ros::transformPointCloud(wrist_frame.str(), scans[i], temp, tf_listener);
        merged += scans[i];
    }
    return merged;
}
void kinectCallback(const PointCloud<PointXYZ>::ConstPtr& in_cloud){
    if (not listen_kinect){
        return;
    }
    listen_kinect = false;
    ROS_INFO("Nube recibida");
    tf::TransformListener tf_listener;
    tf::StampedTransform stamped_tf;
    ros::Time transform_time = ros::Time(0);
    stringstream wrist_frame;
    wrist_frame << GROUP << GRIPPER_FRAME_SUFFIX;
    try{
        ROS_INFO("Esperando transformación...");
        bool got_transform = false;
        for (int i=0; i<TF_RETRYS; i++){
            if (not tf_listener.waitForTransform(in_cloud->header.frame_id, wrist_frame.str(), transform_time, ros::Duration(WAIT_TF_TIMEOUT))){
                    ROS_ERROR("Intento %d de %d: Timeout en %f segundos",(i+1), TF_RETRYS, WAIT_TF_TIMEOUT);
            }
            else{
                got_transform = true;
                break;
            }
        }
        if (not got_transform){
            ROS_ERROR("Error grave: no se pudo obtener transformación entre kinect y el gripper.\nABORTANDO...");
            exit(1);
        }
        tf_listener.lookupTransform(in_cloud->header.frame_id, wrist_frame.str(), ros::Time(0), stamped_tf);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("Exception al obtener transform: %s", ex.what());
    }
    /* Recibo nube de puntos, debo almacenar el escaneo de mi objeto.
    */
    // Eliminar puntos lejanos
    PointCloud<PointXYZ>::Ptr in_cloud_filtered(new PointCloud<PointXYZ>);
    PassThrough<PointXYZ> pass;
    pass.setInputCloud(in_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.6 - passthrough_z/2, 0.6 + passthrough_z/2); //HARDCODEADO. FORMA CORRECTA ES CON TF.
    pass.filter(*in_cloud_filtered);
    
    // Submuestrear
    PointCloud<PointXYZ>::Ptr in_cloud_subsampled(new PointCloud<PointXYZ>);
    VoxelGrid<PointXYZ> subsampler;
    subsampler.setInputCloud(in_cloud_filtered);
    subsampler.setLeafSize(LEAFSIZE, LEAFSIZE, LEAFSIZE);
    subsampler.filter(*in_cloud_subsampled);

    // Aplicar transformación
    PointCloud<PointXYZ> in_cloud_transformed;
    in_cloud_subsampled->header.stamp = stamped_tf.stamp_.toNSec()/1e3;
    pcl_ros::transformPointCloud(wrist_frame.str(), *in_cloud_subsampled, in_cloud_transformed, tf_listener);

    // Guardar la nube de puntos
    ROS_INFO("Añadiendo a listado de scans");
    scans.push_back(in_cloud_transformed);
    // Si ya escaneamos todas las perspectivas, juntarlas, guardarlas y salir
    scan_pose1_orientation[0] += roll_delta;
    if (scan_pose1_orientation[0] > 2*PI){
        PointCloud<PointXYZ> merged = mergeViews();
        // Escribir a archivo pcd
        long int now = (long int)ros::WallTime::now().toSec();
        time_t t = time(NULL);
        struct tm *tm = localtime(&t);
        char date[50];
        strftime(date, sizeof(date), "%d_%m_%Y-%H:%M:%S", tm);
        stringstream filename;
        filename << "Scan_" << date << ".pcd";
        io::savePCDFileASCII(filename.str(), merged);
        // salir
        ROS_INFO("Se ha guardado archivo como '%s'\n", filename.str().c_str());
        exit(0);
    }
    // Poner siguiente pose
    ROS_INFO("Posicionándose para siguiente scan");
    scan_pose1.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(scan_pose1_orientation[0], scan_pose1_orientation[1], scan_pose1_orientation[2]);
    moveToPose(scan_pose1);
    listen_kinect = true;
}
int main(int argc, char **argv){
    // Pedir algun gripper
    if (argc < 2){
        printf("Debe ingresar el gripper a mover: [l|r]\n");
        exit(1);
    }
    if (string(argv[1]).compare("l") != 0 and string(argv[1]).compare("r") != 0){
        printf("Opción inválida: '%c'. Debe ingresar [l|r]\n",*argv[1]);
        exit(1);
    }
    // Inicializar algunas constantes globales
    scan_pose1.header.frame_id = "base_footprint";
    scan_pose1.pose.position.x = scan_pose1_position[0];
    scan_pose1.pose.position.y = scan_pose1_position[1];
    scan_pose1.pose.position.z = scan_pose1_position[2];
    scan_pose1.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(scan_pose1_orientation[0], scan_pose1_orientation[1], scan_pose1_orientation[2]);
    // iniciar un nodo ROS
    ros::init(argc, argv, "gripper_scanner");
    ros::NodeHandle nh;
    signal(SIGINT, signalHandler);
    // Inicializar grupo MoveIt para planear trayectorias
    string group_s = (*argv[1] == 'l' ? "left_arm" : "right_arm");
    ROS_INFO("Iniciando grupo %s", group_s.c_str());
    moveit::planning_interface::MoveGroup group(group_s.c_str());
    gripper_group = &group;
    // "Suscribirse" a servicio de LookAt
    lookat_client = nh.serviceClient<memoria::LookAt>("look_at");
    // Suscribirse al kinect
    ros::Subscriber kinect_sub = nh.subscribe<PointCloud<PointXYZ> > (KINECT_TOPIC, 1, kinectCallback);

    // --- INICIO ---
    ros::AsyncSpinner spinner(1);
    spinner.start();
    // Mirar hacia la pose de escaneo
    lookAt(scan_pose1.header.frame_id, scan_pose1.pose.position.x, scan_pose1.pose.position.y, scan_pose1.pose.position.z + 0.10);
    // Llevar brazo al punto de escaneo
    moveToPose(scan_pose1);
    // Guardar una nube de puntos por cada perspectiva
    listen_kinect = true;
    ros::spin();
/*  
    while (1){
        scan_pose1_orientation[0] += roll_delta;
        if (scan_pose1_orientation[0] > 2*PI)
            scan_pose1_orientation[0] -= 2*PI;
        scan_pose1.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(scan_pose1_orientation[0], scan_pose1_orientation[1], scan_pose1_orientation[2]);
        moveToPose(scan_pose1, gripper_group, planner);
        ros::Duration(0.3).sleep();
    }*/
    return 0;
}