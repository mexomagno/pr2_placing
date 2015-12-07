#include "../Util.h"

// PUBLIC
// CONSTANTES
// Frames
const string Util::BASE_FRAME                  = "/base_footprint";
const string Util::ODOM_FRAME                  = "/odom_combined";
const string Util::CAMERA_FRAME                = "/high_def_frame";
const string Util::KINECT_FRAME                = "/head_mount_kinect_ir_optical_frame";

// Tópicos
const string Util::KINECT_TOPIC                = "head_mount_kinect/depth_registered/points";
const string Util::GRIPPER_GOAL_TOPIC_SUFFIX   = "_gripper_controller/gripper_action/goal";
const string Util::GRIPPER_STATUS_TOPIC_SUFFIX = "_gripper_controller/gripper_action/result";
const string Util::BASE_CONTROLLER_TOPIC       = "/base_controller/command";
const string Util::ODOM_TOPIC                  = "/base_odometry/odom";

// Otros
const float Util::PI  = 3.1416;
const float Util::SUBSAMPLE_LEAFSIZE = 0.05;
// segmentación
const float WAIT_TF_TIMEOUT = 1.0;
const float SEG_THRESHOLD = 0.01;
const float MIN_SEGMENTED_SURFACE_PERCENT = 15;
const float MIN_CLOUD_LEFT_PERCENT = 10;
// Búsqueda de superficie
const float Util::DEFAULT_DESIRED_PITCH = -Util::PI/2.0; // Inclinación deseada del plano (normal vetical)
const float PITCH_THRESHOLD = 0.09;

// MÉTODOS
Util::Util(){}
float Util::toGrad(float rad){
	return rad*180.0/PI;
}
float Util::toRad(float grad){
	return grad*PI/180.0;
}
geometry_msgs::Quaternion Util::coefsToQuaternionMsg(float a, float b, float c){
    /* 
    Recibe: a, b, c, coeficientes de un plano
    Retorna: Quaternion con la orientación de la normal del plano.
    */
    float vmod=sqrt(a*a+b*b+c*c);
    float pitch = -(1.0*asin(1.0*c/vmod));
    float yaw = (a==0?toRad(90):atan(b/a)) + (a>=0?0:toRad(180));
    return tf::createQuaternionMsgFromRollPitchYaw(0,pitch,yaw);
}
/*PointCloud<PointXYZ> Util::scanGripper(char which){

}*/
// Operaciones con nubes de puntos
PointCloud<PointXYZ>::Ptr Util::subsampleCloud(PointCloud<PointXYZ>::Ptr cloud_in, float leafsize){
    PointCloud<PointXYZ>::Ptr cloud_out(new PointCloud<PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> subsampler;
    subsampler.setInputCloud(cloud_in);
    subsampler.setLeafSize(leafsize, leafsize, leafsize);
    subsampler.filter(*cloud_out);
    return cloud_out;
}

geometry_msgs::Point Util::getCloudCentroid(PointCloud<PointXYZ>::Ptr cloud_in){
    Eigen::Vector4f centroid;
    compute3DCentroid(*cloud_in, centroid);
    geometry_msgs::Point point_out;
    point_out.x = centroid(0);
    point_out.y = centroid(1);
    point_out.z = centroid(2);
    return point_out;
}

/**
 * searchPlacingSurface: Función que busca iterativamente en una nube de puntos la ocurrencia de una superficie plana sensata para efectuar el placing.
 *                         Este método debe ser llamado INMEDIATAMENTE luego de obtener la nube de puntos desde el kinect.
 * @param  cloud_in    : Nube de puntos donde buscar
 * @param  min_height  : Altura mínima de la superficie
 * @param  max_height  : Altura máxima de la superficie
 * @param  inclination : Inclinación máxima admitida
 * @return 
 * @param  cloud_out   : Donde se retornan los inliers
 */
bool Util::searchPlacingSurface(PointCloud<PointXYZ>::Ptr cloud_in, PointCloud<PointXYZ>::Ptr &cloud_out, float min_height, float max_height, float inclination){
    // Tamaño inicial de la nube de entrada
    int init_cloud_size = (int)cloud_in->size();
    int current_cloud_size = init_cloud_size;
    tf::TransformListener tf_listener;
    tf::StampedTransform stamped_tf;
    ros::Time transform_time = ros::Time(0);
    // Intentar obtener transformación actual
    try{
        ROS_DEBUG("Util: Esperando transformación disponible...");
        if (not tf_listener.waitForTransform(cloud_in->header.frame_id, BASE_FRAME, transform_time, ros::Duration(WAIT_TF_TIMEOUT))){
            ROS_ERROR("Util: Transformación no pudo ser obtenida antes del timeout (%fs)", WAIT_TF_TIMEOUT);
            return false;
        }
        tf_listener.lookupTransform(cloud_in->header.frame_id, BASE_FRAME, ros::Time(0), stamped_tf);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("Util: Excepción al obtener transformación: %s", ex.what());
    }
    // Preparar para segmentación
    SACSegmentation<PointXYZ> segmentator;
    segmentator.setOptimizeCoefficients(true); // Parece que es redundante
    segmentator.setModelType(SACMODEL_PLANE);
    segmentator.setMethodType(SAC_RANSAC);
    segmentator.setDistanceThreshold(SEG_THRESHOLD);
    // Contenedores para resultados de segmentación
    ModelCoefficients::Ptr coefs (new ModelCoefficients());
    PointIndices::Ptr inliers (new pcl::PointIndices());
    PointCloud<PointXYZ>::Ptr cloud_plane (new PointCloud<PointXYZ>());
    ExtractIndices<PointXYZ> extractor;
    // Iterar, buscando planos hasta encontrar uno, o quedarse con muy pocos puntos
    
    do{
        // Segmentar
        segmentator.setInputCloud(cloud_in);
        segmentator.segment(*inliers, *coefs);
        if (inliers->indices.size()*100.0/current_cloud_size < MIN_SEGMENTED_SURFACE_PERCENT){
            // Si no se encontró el modelo
            if (inliers->indices.size() == 0){
                ROS_INFO("Util: No se encontró plano");
                return false;
            }
            // Si es una superficie muy chica
            ROS_INFO("Util: Superficie muy pequeña (%f%% de un mímino de %f%% del total de puntos)", inliers->indices.size()*100.0/init_cloud_size, MIN_SEGMENTED_SURFACE_PERCENT);
            return false;
        }
        extractor.setInputCloud(cloud_in);
        extractor.setIndices(inliers);
        extractor.setNegative(false);
        extractor.filter(*cloud_plane);
        // Verificar primer criterio de aceptación: Altura mínima
        geometry_msgs::PointStamped cloud_centroid, cloud_centroid_base;
        cloud_centroid.point = Util::getCloudCentroid(cloud_plane);
        cloud_centroid.header.frame_id = cloud_in->header.frame_id;
        cloud_centroid.header.stamp = stamped_tf.stamp_;
        tf_listener.transformPoint(Util::BASE_FRAME, cloud_centroid, cloud_centroid_base);
        if (cloud_centroid_base.point.z > min_height and cloud_centroid_base.point.z < max_height){
            // Primera prueba superada. Ahora, comprobar inclinación
            // Obtener la normal del plano, basada en los coeficientes del plano
            geometry_msgs::QuaternionStamped normal_q, normal_q_base;
            normal_q.quaternion = Util::coefsToQuaternionMsg(coefs->values[0], coefs->values[1], coefs->values[2]);
            normal_q.header.frame_id = cloud_in->header.frame_id;
            normal_q.header.stamp = stamped_tf.stamp_;
            tf_listener.transformQuaternion(Util::BASE_FRAME, normal_q, normal_q_base);
            tf::Quaternion tf_q;
            tf::quaternionMsgToTF(normal_q_base.quaternion, tf_q);
            double roll, pitch, yaw;
            tf::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
            // El siguiente arreglo, puede ser un problema si se encuentra un techo bajo
            pitch = -abs(pitch);
            // Verificar segundo criterio de aceptación: Inclinación máxima
            if (inclination - PITCH_THRESHOLD < pitch and pitch < inclination + PITCH_THRESHOLD){
                ROS_INFO("Util: Superficie encontrada");
                // Guardar superficie encontrada
                extractor.setInputCloud(cloud_in);
                extractor.setIndices(inliers);
                extractor.setNegative(false);
                extractor.filter(*cloud_out);
                return true;
            }
        }
        else {
            ROS_INFO("Util: Plano demasiado %s. Buscando siguiente...", cloud_centroid_base.point.z > max_height ? "alto" : "bajo");
        }
        // Quitar plano encontrado de los puntos restantes
        extractor.setInputCloud(cloud_in);
        extractor.setIndices(inliers);
        extractor.setNegative(true);
        extractor.filter(*cloud_in); // PELIGRO, PUEDE MODIFICAR NUBE ORIGINAL, COSA QUE LA FUNCIÓN NO PRETENDE HACER.
    }while ( ((int)cloud_in->size()*100.0/init_cloud_size) >= MIN_CLOUD_LEFT_PERCENT);
    ROS_INFO("Quedan muy pocos puntos para buscar una superficie. Superficie NO encontrada");
    return false;
}