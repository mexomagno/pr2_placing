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
double scan_pose1_position[] = {0.6, 0, 1.2}; // X, Y, Z
double scan_pose1_orientation[] = {0, -PI/2, 0}; // R, P, Y 
geometry_msgs::PoseStamped scan_pose1;

double roll_delta = PI/2;
const string KINECT_TOPIC = "head_mount_kinect/depth_registered/points";
const string GRIPPER_FRAME_SUFFIX = "_gripper_tool_frame";
// const string GRIPPER_FRAME_SUFFIX = "_wrist_roll_link";
const double WAIT_TF_TIMEOUT = 1.0;
const int TF_RETRYS = 5;
double POST_MOVE_SLEEP = 1;

double colors[10][3];


// VARIABLES GLOBALES
moveit::planning_interface::MoveGroup::Plan planner;
moveit::planning_interface::MoveGroup *gripper_group;
ros::ServiceClient lookat_client;
memoria::LookAt lookat_srv;
char GROUP = 'l'; // Puede ser 'l' o 'r'
bool listen_kinect = false;
vector<PointCloud<PointXYZ> > scans;

// Variables auxiliares
ros::Publisher scan1_publisher;
ros::Publisher scan2_publisher;
ros::Publisher scan3_publisher;
ros::Publisher scan4_publisher;
int curr_pub = 1;

// METODOS

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
PointCloud<PointXYZRGB> mergeViews(){
    /* Recorre arreglo de pointclouds y retorna sólo una, mezclada */
    PointCloud<PointXYZRGB> merged;
    merged.height = 1;
    merged.width = 0;
    // Agregar primer elemento
    // Rotar según eje X en sentido contrario de rotación según roll_delta.
    // Notar que scans[i] está rotado en (roll_delta*(i+1))
    // Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
    // double angle = 0;//roll_delta;
    // rotation(1,1) = cos(angle);
    // rotation(1,2) = -sin(angle);
    // rotation(2,1) = sin(angle);
    // rotation(2,2) = cos(angle);
    // Aplicar rotación
    // PointCloud<PointXYZ>::Ptr scan_rotated_ptr (new PointCloud<PointXYZ>());
    // transformPointCloud(scans[0], *scan_rotated_ptr, rotation);
    //merged = scans[0];
    for (int i=0; i<scans.size(); i++){
        merged.width += scans[i].points.size();
        for (int j=0; j<scans[i].points.size(); j++){
            PointXYZ scanpoint = scans[i].points[j];
            int r, g, b;
            switch (i){
                case 0:
                    r=255; g=b=0;
                    break;
                case 1:
                    r=b=0; g=255;
                    break;
                case 2: 
                    r=g=0; b=255;
                    break;
                case 3:
                    r=g=255; b=0;
                    break;
                case 4:
                    r=b=255; g=0;
                    break;
                case 5:
                    r=0; g=b=255;
                    break;
                case 6:
                    r=g=b=255;
                    break;
                case 7:
                    r=g=b=0;
                    break;
                default:
                    r = (int)(rand()%255);
                    g = (int)(rand()%255);
                    b = (int)(rand()%255);
            }
            PointXYZRGB newpoint = PointXYZRGB(r, g, b);
            newpoint.x = scanpoint.x;
            newpoint.y = scanpoint.y;
            newpoint.z = scanpoint.z;
            merged.points.push_back(newpoint);
        }
        // Cambiar ángulo
        // double angle = 0;//(roll_delta*(i+1));
        // rotation(1,1) = cos(angle);
        // rotation(1,2) = -sin(angle);
        // rotation(2,1) = sin(angle);
        // rotation(2,2) = cos(angle);
        // Aplicar rotación
        // transformPointCloud(scans[i], *scan_rotated_ptr, rotation);
    //    merged += scans[i];
    }
    // Borrar gripper
    PointCloud<PointXYZRGB>::Ptr merged_filtered(new PointCloud<PointXYZRGB>()), merged_ptr(new PointCloud<PointXYZRGB>());
    *merged_ptr = merged;
    PassThrough<PointXYZRGB> pass;
    pass.setInputCloud(merged_ptr);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-0.14, 1000);
    pass.filter(*merged_filtered);
    return *merged_filtered;
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
    ROS_INFO("Nube recibida es de T=%f", (float)(in_cloud->header.stamp/1e6));
    ROS_INFO("Transform obtenido es de T=%f", (float)(stamped_tf.stamp_.toNSec()/1e9));

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


    switch (curr_pub){
        case 1:
            scan1_publisher.publish(in_cloud);
            break;
        case 2:
            scan2_publisher.publish(in_cloud);
            break;
        case 3:
            scan3_publisher.publish(in_cloud);
            break;
        case 4:
            scan4_publisher.publish(in_cloud);
            break;
    }
    curr_pub++;
    // Si ya escaneamos todas las perspectivas, juntarlas, guardarlas y salir
    scan_pose1_orientation[0] += roll_delta;
    if (scan_pose1_orientation[0] > 2*PI){
        PointCloud<PointXYZRGB> merged = mergeViews();
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
    ros::Duration(POST_MOVE_SLEEP).sleep();
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
    GROUP = string(argv[1]).compare("l") == 0 ? 'l' : 'r';
    if (argc >= 3){
        roll_delta = atof(argv[2])*PI/180.0;
    }
    if (argc >= 4){
        POST_MOVE_SLEEP = atof(argv[3]);
    }
    ROS_INFO("Usando roll_delta = %f°",(roll_delta*180/PI));
    ROS_INFO("Usando POST_MOVE_SLEEP = %f", POST_MOVE_SLEEP);
    // Inicializar algunas constantes globales
    scan_pose1.header.frame_id = "base_footprint";
    scan_pose1.pose.position.x = scan_pose1_position[0];
    scan_pose1.pose.position.y = scan_pose1_position[1];
    scan_pose1.pose.position.z = scan_pose1_position[2];
    scan_pose1_orientation[0] += roll_delta;
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
    // Publicar auxiliares
    scan1_publisher = nh.advertise<PointCloud<PointXYZ> > ("scan_1", 1);
    scan2_publisher = nh.advertise<PointCloud<PointXYZ> > ("scan_2", 1);
    scan3_publisher = nh.advertise<PointCloud<PointXYZ> > ("scan_3", 1);
    scan4_publisher = nh.advertise<PointCloud<PointXYZ> > ("scan_4", 1);


    // --- INICIO ---
    ros::AsyncSpinner spinner(1);
    spinner.start();
    // Mirar hacia la pose de escaneo
    lookAt(scan_pose1.header.frame_id, scan_pose1.pose.position.x, scan_pose1.pose.position.y, scan_pose1.pose.position.z + 0.10);
    // Llevar brazo al punto de escaneo
    moveToPose(scan_pose1);
    ros::Duration(POST_MOVE_SLEEP).sleep();
    // Guardar una nube de puntos por cada perspectiva
    listen_kinect = true;
    ros::spin();
    return 0;
}
/*
    TODO:   
        [DONE]- Estabilizar scaneo: Se ven muy separadas las capas.
            - Puede deberse a que el gripper no sea totalmente simétrico...
            - Puede ser porque se está tomando un scan del kinect un poco atrasado, justo antes de llegar al punto indicado
            - Habrá problemas con el transform?

            Observaciones: 
                - Cada scan parece estar corrido hacia la izquierda
                - En Rviz, el tópico del kinect se visualiza perfecto, pero cada scan por separado se ve corrido hacia la izquierda (del robot)
                    Esto era causado por usar "depth_registered" para visualizar en rviz, pero "depth" para el programa.
        - Filtrar el gripper
        - 
*/