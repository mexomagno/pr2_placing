/*
Servicio encargado de encontrar superficie de placing
Recibe: Nada
Retorna: - sensor_msgs/PointCloud2
         - memoria/ErrorMsg
*/

#include <string>
#include <vector>
#include <csignal>
//#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <memoria/SearchSurface.h>
#include <sensor_msgs/PointCloud2.h>
#include <memoria/ErrorMsg.h>
#include <memoria/LookAt.h>
#include <memoria/LookAtMsg.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>

using namespace std;
// Acortar nombre del tipo pointcloud
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// CONSTANTES
const string SERVICE_NAME = "search_surface";
const string ROBOT_FRAME = "/base_footprint";
const string WORLD_FRAME = "/odom_combined";
const string KINECT_TOPIC = "head_mount_kinect/depth/points";
const float WAIT_LOOKAT = 1;
const float PI = 3.1415;
const float HEAD_YAWS[] = {-PI*2/4.0, -PI*1/4.0, 0, PI*1/4.0, PI*2/4}; // Posiciones de cabeza hardcodeadas para búsqueda de superficies.
const float PITCH_THRESHOLD = 0.09; // Holgura de inclinación de superficie, en radianes
const float DESIRED_PITCH = PI/-2.0; // Inclinación deseada del plano (normal vertical)
const float MIN_SURFACE_HEIGHT = 0.3; // Altura mínima de la superficie al suelo, en metros.
const int HEAD_YAWS_SIZE = (int)(sizeof(HEAD_YAWS)/sizeof(*HEAD_YAWS));
// Parámetros
//      de submuestreo
const double LEAFSIZEX = 0.05, LEAFSIZEY = 0.05, LEAFSIZEZ = 0.05; 
//      de segmentación
const float SEG_THRESHOLD = 0.01; // Radio para considerarse inlier
const float MIN_CLOUD_LEFT_RATIO = 10; // Mínimo porcentaje de nube restante para seguir buscando una superficie
const float MIN_SEGMENTED_SURFACE_PERCENT = 10; // Tamaño mínimo de superficie en términos de porcentaje de puntos versus puntos totales de la escena inicialmente capturada
//      de transformaciones TF
const float WAIT_TF_TIMEOUT = 1.0;

// VARIABLES GLOBALES
ros::ServiceClient lookat_client;
memoria::LookAt lookat_srv;
bool surface_found = false; // True si se encuentró una superficie.
pcl::SACSegmentation<pcl::PointXYZ> segmentator;
sensor_msgs::PointCloud2 selected_surface_msg;
PointCloud::Ptr selected_surface (new PointCloud);
bool service_called = false;
// Códigos de error
vector<string> errors(5);

// Variables Auxiliares
ros::Publisher aux_pointcloud_publisher;
ros::Publisher aux_pointcloud_publisher_2;
ros::Publisher aux_pose_publisher;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// Métodos
void signalHandler( int signum ){
    exit(0);
}
float toGrad(float rad){
    return rad*360/(2*3.1415);
}
float toRad(int grados){
    return grados*2*3.1415/360.0;
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
void kinectCallback(const PointCloud::ConstPtr& in_cloud){
    if (not service_called){
        return;
    }
    ROS_INFO("Nube recibida");
    // ********* Obtener transformación de este momento con TF
    // Crear objetos
    tf::TransformListener tf_listener;
    tf::StampedTransform stamped_tf;
    ros::Time transform_time = ros::Time(0);
    // Intentar obtener transformación actual
    try{
        ROS_INFO("Esperando transformación disponible...");
        if (not tf_listener.waitForTransform(in_cloud->header.frame_id, ROBOT_FRAME, transform_time, ros::Duration(WAIT_TF_TIMEOUT))){
            ROS_ERROR("Transformación no pudo ser obtenida en %f segundos",WAIT_TF_TIMEOUT);
            // Salir del callback
            return;
        }
        tf_listener.lookupTransform(in_cloud->header.frame_id, ROBOT_FRAME, ros::Time(0), stamped_tf);
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
        centroid_msg.header.stamp = stamped_tf.stamp_;
        centroid_msg.point.x = centroid(0);
        centroid_msg.point.y = centroid(1);
        centroid_msg.point.z = centroid(2);
        tf_listener.transformPoint(ROBOT_FRAME, centroid_msg, centroid_msg_baseframe);
        ROS_INFO("Centroide en (ROBOT_FRAME): (%f,%f,%f)",centroid_msg_baseframe.point.x,centroid_msg_baseframe.point.y,centroid_msg_baseframe.point.z);
        // ********** VERIFICAR 1-ER CRITERIO ACEPTACIÓN: altura del plano
        if (centroid_msg_baseframe.point.z > MIN_SURFACE_HEIGHT){
            // Obtener la normal del plano
            geometry_msgs::QuaternionStamped normal_quat,normal_quat_baseframe;
            normal_quat.quaternion = coefsToQuaternionMsg(coefs->values[0],coefs->values[1],coefs->values[2]);
            normal_quat.header.frame_id = in_cloud->header.frame_id;
            //normal_quat.header.stamp = ros::Time(in_cloud->header.stamp/1000000.0);//stamped_tf.stamp_;
            normal_quat.header.stamp = stamped_tf.stamp_;
            //      Transformar normal a frame de la base
            tf_listener.transformQuaternion(ROBOT_FRAME,normal_quat,normal_quat_baseframe);
            //      Expresarla en términos de RPY
            tf::Quaternion tf_quaternion;
            tf::quaternionMsgToTF(normal_quat_baseframe.quaternion,tf_quaternion);
            double roll,pitch,yaw;
            tf::Matrix3x3(tf_quaternion).getRPY(roll,pitch,yaw);
            ROS_INFO("Normal en RPY (ROBOT_FRAME): (%f°, %f°, %f°)",toGrad(roll),toGrad(pitch),toGrad(yaw));
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
            ROS_INFO("Plano demasiado bajo (altura: %f, mín: %f)",centroid_msg_baseframe.point.z,MIN_SURFACE_HEIGHT);
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

int getSurface(PointCloud::Ptr& pointcloud){
    /*
    Recibe contenedor de pointcloud encontrada.
    Utiliza callback del tópico suscrito al kinect para buscar la nub.
    */
    ROS_INFO("Mirando un poco hacia el suelo");
    if (not lookAt(ROBOT_FRAME,2,0,0,false)){
        return 4;
    }
    // Mirar al primer punto de búsqueda de superficie
    int i=0;
    bool vuelta = false;
    while (i>=0){//for (int i = 0; i< HEAD_YAWS_SIZE; i++){
        ROS_INFO("Buscando en %f°",toGrad(HEAD_YAWS[i]));
        lookAt(ROBOT_FRAME,HEAD_YAWS[i],0,0,true);
        ros::Duration(WAIT_LOOKAT).sleep();
        ros::spinOnce();
        if (surface_found){
            ROS_INFO("Se almacena superficie encontrada.");
            pointcloud = selected_surface;
            break;
        }
        i = vuelta ? i-1 : i+1;
        if (i==HEAD_YAWS_SIZE){
            ROS_INFO("Primera vuelta infructuosa. Elevando la cabeza y reintentando...");
            vuelta = true;
            i--;
            lookAt(ROBOT_FRAME,0,1,1.3,false);
        }

    }
    if (not surface_found){
        ROS_INFO("No se encontró ninguna superficie adecuada. Abortando...");
        return 1;
    }
    else{
        ROS_INFO("Fin búsqueda superficie");
        surface_found = false;
        return 0;
    }
}
int getDummyCloud(PointCloud::Ptr& pointcloud){
    PointCloud::Ptr cloud (new PointCloud);
    cloud->header.frame_id = "head_mount_kinect_ir_optical_frame";
    cloud->width = 20;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);
    for (size_t i = 0; i < cloud->points.size(); i++){
        double factor = 0.1;
        cloud->points[i].x = 10+(i%5)*factor;
        cloud->points[i].z = 10+floor(i/5)*factor;
        cloud->points[i].y = 5+1 + 1.0*(rand()/RAND_MAX)*factor;
    }
    ROS_INFO("Construí nube dummy de %d puntos", (int)cloud->size());
    pointcloud = cloud;
    return 0;
}

// ---------------------------------------------
int searchSurface(){
    int retcode;
    // Buscar superficie
    ROS_INFO("Buscando superficie");
    PointCloud::Ptr cloud (new PointCloud);
    retcode = getSurface(cloud);
    //retcode = getDummyCloud(cloud);
    if (retcode != 0){
        return retcode;
    }
    pcl::toROSMsg(*cloud, selected_surface_msg);
    // Guardar superficie encontrada
    //ROS_INFO("Encontrada. Guardando...");
    //getDummyCloud(cloud);
    ROS_INFO("Obtenida nube de %d puntos",(int)cloud->size());
    return retcode;
}
bool callback(memoria::SearchSurface::Request& request, memoria::SearchSurface::Response& response){
    ROS_INFO("Request entrante");
    service_called = true;
    // Buscar superficie
    int retcode = searchSurface();
    // Almacenar superficie
    response.pointcloud = selected_surface_msg;
    // Crear código de error
    memoria::ErrorMsg errormsg;
    errormsg.retcode = retcode;
    errormsg.what = errors[errormsg.retcode];
    response.error = errormsg;
    service_called = false;
    return true;
}
int main(int argc, char **argv){
    errors[0]="";
    errors[1]="Superficie no encontrada";
    errors[2]="Servicio 'LookAt' no disponible";
    errors[3]="Error desconocido";
    errors[4]="Error al llamar al servicio 'look_at'";
    ROS_INFO("Iniciando servicio '%s'",SERVICE_NAME.c_str());
    ros::init(argc, argv, "search_surface_server");
    ros::NodeHandle nh;
    signal(SIGINT, signalHandler);
    // "Subscribirse" al servicio look_at
    lookat_client = nh.serviceClient<memoria::LookAt>("look_at");
    // Publicadores auxiliares
    aux_pointcloud_publisher = nh.advertise<PointCloud> ("plano_submuestreado", 1);
    aux_pointcloud_publisher_2 = nh.advertise<PointCloud> ("superficie_encontrada", 1);
    aux_pose_publisher = nh.advertise<PointCloud> ("normal_plano", 1);
    // Suscribir al kinect
    ros::Subscriber kinect_sub = nh.subscribe<PointCloud>(KINECT_TOPIC, 1, kinectCallback);
    // Crear servicio
    ros::ServiceServer service = nh.advertiseService(SERVICE_NAME,callback);
    ROS_INFO("Listo para recibir requests");
    ros::spin();
    return 0;
}