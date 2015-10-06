/* 
Este algoritmo se basa en el flujo explicitado en "~/Escritorio/Memoria/Conceptos"
Pseudocódigo:
    -
    -Apuntar cabeza un poco al suelo
    -Buscar superficie:
        - Calzar modelo con RANSAC (plano primeramente)
        - if encontrado:
            - Aplicar restricciones
            - if cumple:
                return ENCONTRADO
        - NO ENCONTRADO
        if no encontrado:
            - buscar en otro lado
            - if busqué todo a mi alcance
                - return NO ENCONTRADO (ABORTAR)

Componentes necesarias:
LIBRERÍAS
    ROS
        ros/ros : para todo ros
        Mensajes:
            sensor_msgs/PointCloud2 : para recibir mensajes de kinect
            geometry_msgs/PointStamped : Para enviar requests al server de la cabeza
        memoria/LookAt :  Para usar el servicio lookat para control de la cabeza
        memoria/LookAtMsg :  Mensaje personalizado para el server de control de la cabeza
    PCL para procesamiento de nube de puntos del Kinect
        pcl/point_cloud : para usar clase pointcloud
        pcl/point_types : para usar tipos de puntos
        pcl/segmentation/sac_segmentation : para segmentacion
        pcl/filters/voxel_grid : Para submuestreo
        pcl/filters/extract_indices : para extracción de índices de inliers
        pcl/features/normal_3d : Orientación de vectores hacia viewpoint
        // pcl_conversions/pcl_conversions & 
        pcl_ros/point_cloud: para trabajar con tipos de datos de pcl directamente
        pcl/common/centroid : Para obtención de centroide de una nube de puntos.

    TF para transformaciones de frames
        tf/transform_listener : listener de transformaciones
    OTROS
        math : para operaciones matemáticas trigonométricas
        iostream : para usar <<
        string : Strings de más alto nivel que char *

TODO:
    - Usar opciones de Sacsegmentation para filtrar normales desde un principio (setAxis, segEpsAngle)
    - Evaluar la implementación de un mecanismo más exacto de detección de altura del plano.

*/
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <memoria/LookAt.h>
#include <memoria/LookAtMsg.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/centroid.h>
//#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <iostream>
#include <string>

// Ahorrarse el prefijo "std"
using namespace std;
// Acortar nombre del tipo pointcloud
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// CONSTANTES
const string KINECT_TOPIC = "head_mount_kinect/depth/points";
const string KINECT_FRAME = "head_mount_kinect_ir_optical_frame";
const string QUERY_TOPIC = "memoria/lookat";
const string WORLD_FRAME = "/odom_combined";
const string ROBOT_BASE_FRAME = "/base_footprint";
const double pi = 3.14159;
const float HEAD_YAWS[] = {-pi*2/4.0, -pi*1/4.0, 0, pi*1/4.0, pi*2/4}; // Posiciones de cabeza hardcodeadas para búsqueda de superficies.
const int HEAD_YAWS_SIZE = (int)(sizeof(HEAD_YAWS)/sizeof(*HEAD_YAWS));
const float WAIT_LOOKAT = 1;
const float PITCH_THRESHOLD = 0.09; // Holgura de inclinación de superficie, en radianes
const float DESIRED_PITCH = -3.14159/2.0; // Inclinación deseada del plano (normal vertical)
const float MIN_SURFACE_HEIGHT = 0.3; // Altura mínima de la superficie al suelo, en metros.
// Parámetros
//      de submuestreo
const double LEAFSIZEX = 0.05, LEAFSIZEY = 0.05, LEAFSIZEZ = 0.05; 
//      de segmentación
const float SEG_THRESHOLD = 0.01; // Radio para considerarse inlier
const float MIN_CLOUD_LEFT_RATIO = 10; // Mínimo porcentaje de nube restante para seguir buscando una superficie
const float MIN_SEGMENTED_SURFACE_PERCENT = 10; // Tamaño mínimo de superficie en términos de porcentaje de puntos versus puntos totales de la escena inicialmente capturada
//      de transformaciones TF
const float WAIT_TRANSFORM_TIMEOUT = 1.0;

// VARIABLES GLOBALES
ros::ServiceClient lookat_client;
memoria::LookAt lookat_srv;
pcl::SACSegmentation<pcl::PointXYZ> segmentator;
PointCloud::Ptr selected_surface (new PointCloud);
bool surface_found = false;
// Variables auxiliares
ros::Publisher aux_pointcloud_publisher;
ros::Publisher aux_pointcloud_publisher_2;
ros::Publisher aux_pose_publisher;

// MÉTODOS
float toGrad(float rad){
    return rad*360/(2*3.1415);
}
float toRad(int grados){
    return grados*2*3.1415/360.0;
}
bool lookAt(string frame_id, double x, double y, double z, bool rotate){
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
/*tf::Vector3 PlaneNormalToCoefs(geometry_msgs::Pose normal){
    double a=
}
*/
void kinectCallback(const PointCloud::ConstPtr& in_cloud){
    ROS_INFO("Nube recibida");
    // ********* Obtener transformación de este momento con TF
    // Crear objetos
    tf::TransformListener transform_listener;
    tf::StampedTransform stamped_transform;
    ros::Time transform_time = ros::Time(0);
    // Intentar obtener transformación actual
    try{
        ROS_INFO("Esperando transformación disponible...");
        if (not transform_listener.waitForTransform(in_cloud->header.frame_id, ROBOT_BASE_FRAME, transform_time, ros::Duration(WAIT_TRANSFORM_TIMEOUT))){
            ROS_ERROR("Transformación no pudo ser obtenida en %f segundos",WAIT_TRANSFORM_TIMEOUT);
            // Salir del callback
            return;
        }
        transform_listener.lookupTransform(in_cloud->header.frame_id, ROBOT_BASE_FRAME, ros::Time(0), stamped_transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("Excepción al obtener transformación: %s",ex.what());
    }
    // ********* Submuestrear nube para procesamiento más rápido
    PointCloud::Ptr in_cloud_subsampled(new PointCloud), // Subsampleada
                    segmented(new PointCloud),           // Nube inliers
                    remaining_cloud (new PointCloud);
    pcl::VoxelGrid<pcl::PointXYZ> subsampler;
    subsampler.setInputCloud(in_cloud);
    subsampler.setLeafSize (LEAFSIZEX,LEAFSIZEY,LEAFSIZEZ);
    subsampler.filter(*in_cloud_subsampled);
    ROS_INFO("Subsampleo: %d puntos de %d (%f%%)",(int)in_cloud_subsampled->size(),(int)in_cloud->size(),(100.0*(int)in_cloud_subsampled->size()/(int)in_cloud->size()));
    aux_pointcloud_publisher.publish(in_cloud_subsampled);
    // ********* Segmentar en búsqueda de un plano
    segmentator.setOptimizeCoefficients(true);
    segmentator.setModelType(pcl::SACMODEL_PLANE);
    segmentator.setMethodType(pcl::SAC_RANSAC);
    segmentator.setDistanceThreshold(SEG_THRESHOLD);
    // Contenedores de resultados de segmentación
    pcl::ModelCoefficients::Ptr coefs (new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZ> extractor;
    // opcional: segmentator.setMaxIterations(1000);
    // ********* Iterar sobre cada plano encontrado hasta cierto criterio
    // Criterio: Hasta que puntos restantes sean menores a un porcentaje de los puntos iniciales
    //           o hasta que el modelo calzado actual tenga menos de X inliers
    int init_cloud_size = (int)in_cloud_subsampled->size();
    do{
        // ********* Segmentar
        segmentator.setInputCloud(in_cloud_subsampled);
        segmentator.segment(*inliers,*coefs);
        if (inliers->indices.size()*100.0/(int)in_cloud_subsampled->size() < MIN_SEGMENTED_SURFACE_PERCENT){
            // Si no se encuentra superficie
            if (inliers->indices.size() == 0)
                ROS_INFO("No encontré más superficies");
            else
            // Superficie muy chica
                ROS_INFO("Superficie muy pequeña (<%f%% del total de puntos)",inliers->indices.size()*100.0/init_cloud_size);
            return;
        }
        ROS_INFO("Coeficientes plano encontrado (KINECT_FRAME): (%f, %f, %f)",coefs->values[0],coefs->values[1],coefs->values[2]);
        // Extraer inliers
        extractor.setInputCloud(in_cloud_subsampled);
        extractor.setIndices(inliers);
        extractor.setNegative(false);
        extractor.filter(*segmented);
        // ********* VERIFICAR 1-ER CRITERIO ACEPTACIÓN: Altura de la superficie
        /*
        Cómo implementar esta parte?
            - Altura del centroide del plano
                Pros: Simple, asegura al menos un punto de apoyo
                Contras: Centroide puede estar bajo altura mínima pero superficie puede servir igual
            - Altura del punto más bajo
                Pros: Asegura superficie útil
                Contras: Descarta potencialmente otras también útiles (caso anterior)
            - Altura del punto más alto
                Pros: none
                Contras: Todos.
            [ELEGIDA]- Existencia de suficientes puntos sobre cierta altura deseada
                Pros: Asegura superficie de apoyo. Manera más correcta.
                Contras: Cómo se implementa? Hay que transformar TODOS los inliers al frame de la base.
                Implementación propuesta (luego de googlear opciones http://www.pcl-users.org/Filtering-points-above-below-a-plane-td4037613.html):
                    - Crear nueva nube
                    - Crear coefs de planos que restrinjan eje Z según FRAME_ROBOT
                    - Transformar estos coefs a FRAME_KINECT
                    - Iterar sobre los puntos segmentados
                        - if punto entre los planos, añadir a la nueva nube
                    - comprobar nube resultante
                        - if superficie es de buen tamaño, ok.

                Implementación Fácil pero lenta:
                    - Transformar nube segmentada a ROBOT_FRAME
                    - Iterar sobre puntos
                        - if punto bajo Z, remover
                Implementación Fácil, rápida pero poco exacta
                    - Obtener centroide nube segmentada (o cualquier punto)
                    - Transformar centroide a ROBOT_FRAME
                    - if pose > min z, aceptada.

        */
        // Obtener centroide
        Eigen::Vector4f centroid;
        compute3DCentroid(*segmented,centroid);
        // Transformarlo a ROBOT_FRAME
        geometry_msgs::PointStamped centroid_msg,centroid_msg_baseframe;
        centroid_msg.header.frame_id = in_cloud->header.frame_id;
        centroid_msg.header.stamp = stamped_transform.stamp_;
        centroid_msg.point.x = centroid(0);
        centroid_msg.point.y = centroid(1);
        centroid_msg.point.z = centroid(2);
        transform_listener.transformPoint(ROBOT_BASE_FRAME, centroid_msg, centroid_msg_baseframe);
        ROS_INFO("Centroide en (ROBOT_BASE_FRAME): (%f,%f,%f)",centroid_msg_baseframe.point.x,centroid_msg_baseframe.point.y,centroid_msg_baseframe.point.z);
        // ********** VERIFICAR 1-ER CRITERIO ACEPTACIÓN: altura del plano
        if (centroid_msg_baseframe.point.z > MIN_SURFACE_HEIGHT){
            // Obtener la normal del plano
            geometry_msgs::QuaternionStamped normal_quat,normal_quat_baseframe;
            normal_quat.quaternion = coefsToQuaternionMsg(coefs->values[0],coefs->values[1],coefs->values[2]);
            normal_quat.header.frame_id = in_cloud->header.frame_id;
            //normal_quat.header.stamp = ros::Time(in_cloud->header.stamp/1000000.0);//stamped_transform.stamp_;
            normal_quat.header.stamp = stamped_transform.stamp_;
            //      Transformar normal a frame de la base
            transform_listener.transformQuaternion(ROBOT_BASE_FRAME,normal_quat,normal_quat_baseframe);
            //      Expresarla en términos de RPY
            tf::Quaternion tf_quaternion;
            tf::quaternionMsgToTF(normal_quat_baseframe.quaternion,tf_quaternion);
            double roll,pitch,yaw;
            tf::Matrix3x3(tf_quaternion).getRPY(roll,pitch,yaw);
            ROS_INFO("Normal en RPY (ROBOT_BASE_FRAME): (%f°, %f°, %f°)",toGrad(roll),toGrad(pitch),toGrad(yaw));
            // ********* VERIFICAR 2-DO CRITERIO ACEPTACIÓN: inclinación (pitch)
            if (DESIRED_PITCH - PITCH_THRESHOLD < pitch and pitch < DESIRED_PITCH + PITCH_THRESHOLD ){
                ROS_INFO("Superficie encontrada!");
                // Guardar superficie encontrada
                extractor.setInputCloud(in_cloud_subsampled);
                extractor.setIndices(inliers);
                extractor.setNegative(false);
                extractor.filter(*selected_surface);
                // Publicar para visualización
                aux_pointcloud_publisher_2.publish(selected_surface);
                surface_found = true;
                // Terminar callback
                return;
            }
            else{
                ROS_INFO("Superficie demasiado inclinada (%frad demás)",(pitch - DESIRED_PITCH));
            }
        }
        else {
            ROS_INFO("Plano demasiado bajo (altura: %f, mín: $f)",centroid_msg_baseframe.point.z,MIN_SURFACE_HEIGHT);
        }
        // Quitar inliers rechazados para la siguiente iteración
        // "in_cloud -= inliers"
        extractor.setInputCloud(in_cloud_subsampled);
        extractor.setIndices(inliers);
        extractor.setNegative(true);
        extractor.filter(*in_cloud_subsampled);
    }while ( ((int)in_cloud_subsampled->size()*100.0/init_cloud_size) >= MIN_CLOUD_LEFT_RATIO);
    // Al parecer lo anterior debiera ser un while true, que muere en la condición de los inliers de segmentacioń
    ROS_INFO("Quedan muy pocos puntos para buscar una superficie. Superficie NO ENCONTRADA.");
}

int main(int argc, char **argv){
    // Iniciar ROS
    ROS_INFO("Comenzando programa");
    ros::init(argc, argv, "memoria");
    ros::NodeHandle nh;
    // Suscribirse al Kinect
    ros::Subscriber kinect_sub = nh.subscribe<PointCloud>(KINECT_TOPIC, 1, kinectCallback);
    // "Subscribirse" al servicio de LookAt
    lookat_client = nh.serviceClient<memoria::LookAt>("look_at");
    // Publicador auxiliar
    aux_pointcloud_publisher = nh.advertise<PointCloud> ("plano_submuestreado", 1);
    aux_pointcloud_publisher_2 = nh.advertise<PointCloud> ("superficie_encontrada", 1);
    aux_pose_publisher = nh.advertise<PointCloud> ("normal_plano", 1);
    // Mirar un poco al suelo (por ahi por 5,0,0 respecto al robot)
    ROS_INFO("Mirando un poco hacia el suelo");
    if (not lookAt(ROBOT_BASE_FRAME,2,0,0,false)){
        return 1;
    }
    // Mirar al primer punto de búsqueda de superficie
    for (int i = 0; i< HEAD_YAWS_SIZE; i++){
        ROS_INFO("Buscando en %f°",toGrad(HEAD_YAWS[i]));
        lookAt(ROBOT_BASE_FRAME,HEAD_YAWS[i],0,0,true);
        ros::Duration(WAIT_LOOKAT).sleep();
        ros::spinOnce();
        if (surface_found){
            ROS_INFO("Se almacena superficie encontrada.");
            break;
        }
    }
    if (not surface_found){
        ROS_INFO("No se encontró ninguna superficie adecuada. Abortando...");
    }
    return 0;
}