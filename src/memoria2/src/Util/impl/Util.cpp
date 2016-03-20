#include "../Util.h"

// PUBLIC
// CONSTANTES
// Frames
const string Util::BASE_FRAME                  = "/base_footprint";
const string Util::ODOM_FRAME                  = "/odom_combined";
const string Util::CAMERA_FRAME                = "/high_def_frame";
const string Util::KINECT_FRAME                = "/head_mount_kinect_rgb_optical_frame";
const string Util::TORSO_FRAME                 = "/torso_lift_link";
const string Util::GRIPPER_FRAME_SUFFIX        = "_gripper_tool_frame";

// Tópicos
const string Util::KINECT_TOPIC                = "/head_mount_kinect/depth_registered/points";
const string Util::KINECT_TOPIC_SELF_FILTERED  = "/move_group/moveit_self_filtered";
const string Util::GRIPPER_GOAL_TOPIC_SUFFIX   = "_gripper_controller/gripper_action/goal";
const string Util::GRIPPER_STATUS_TOPIC_SUFFIX = "_gripper_controller/gripper_action/result";
const string Util::BASE_CONTROLLER_TOPIC       = "/base_controller/command";
const string Util::ODOM_TOPIC                  = "/base_odometry/odom";

// Strings en general
const string Util::COLLISION_BALL_ID           = "object_collision_ball"; 
const string Util::COLLISION_MESH_ID           = "object_collision_mesh"; 
const string Util::COLLISION_SURFACE_ID        = "surface_collision_mesh";
const string Util::GRIPPER_JOINT_PREFIX        = "_gripper_joint";
const string Util::GRIPPER_LINK_PREFIX         = "_wrist_roll_link";

// Otros
const float Util::PI                      = 3.1416;
const float Util::SUBSAMPLE_LEAFSIZE      = 0.05;
const int   FIND_SURFACE_POSE_RETRYS      = 3;
// segmentación
const float WAIT_TF_TIMEOUT               = 1.0;
const float SEG_THRESHOLD                 = 0.01;
const float MIN_SEGMENTED_SURFACE_PERCENT = 5;
const float MIN_CLOUD_LEFT_PERCENT        = 10;
// Búsqueda de superficie
const float Util::DEFAULT_DESIRED_PITCH   = -Util::PI/2.0; // Inclinación deseada del plano (normal vetical)
const float PITCH_THRESHOLD               = 0.09;
const float Util::KINECT_STABILIZE_TIME   = 1;
const float Util::ROBOT_FRONT_MARGIN      = 0.5;
const float Util::SURFACE_REFINING_THRESHOLD    = 0.02;
const int   Util::SURFACE_REFINING_ITERATIONS   = 3;
const float Util::active_gripper_starting_position[] = {0, 0.7, 0.7};
const float Util::active_gripper_starting_orientation[] = {0, -Util::PI/2.0, 0};

// Gripper Scanner
const float Util::SCAN_ROLL_DELTA         = Util::PI/2.0;
const float Util::scan_position[]         = {0.68, 0, 0.01}; // RELATIVO A TORSO
const float Util::scan_orientation[]      = {0, -Util::PI/2.0, 0}; //R, P, Y
const float Util::tuck_position[]         = {0, 0.40, 0.48}; // Relativo a baseframe
const float Util::tuck_orientation[]      = {Util::PI/2.0, Util::PI/2.0, 0}; // relativo a baseframe
const float Util::GRIPPER_STABILIZE_TIME  = 1;
const float Util::SCAN_PASSTHROUGH_Z      = 0.5;
const float Util::SCAN_LEAFSIZE           = 0.005;
float       Util::COLLISION_BALL_RADIUS   = 0.20; // Radio de bola protectora de gripper
const float Util::VOXEL_UPDATE_DELAY      = 1;
// Stable surface
const float Util::PATCH_ANGLE_THRESHOLD  = 0.2;
// Placing 
const float Util::PLACING_Z_MARGIN        = 0.01; // [Mejor funcional: 0.08] Distancia desde el objeto a la superficie, donde soltarlo. MOVER CON CUIDADO PARA NO CHOCAR CON OCTOMAP
const float Util::PLACING_BACKOFF_DISTANCE = 0.15; // Distancia hacia la que retroceder cuando se suelta el objeto


// Simples variables
bool get_world = false;
vector<moveit_msgs::CollisionObject> objects;
vector<moveit_msgs::AttachedCollisionObject> attached_objects;

// MÉTODOS
Util::Util(){}
// Conversiones
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
float Util::angleBetweenVectors(float x1, float y1, float z1, float x2, float y2, float z2){
    Eigen::Vector3f v1(x1, y1, z1),
                    v2(x2, y2, z2);
    float angle = acos(v1.normalized().dot(v2.normalized()));
    if (angle > Util::PI)
        ROS_INFO("UTIL: WARNING: angulo entre vectores mayor a 180 (%f)", Util::toGrad(angle));
    return angle;
}
Eigen::Vector3f Util::quaternionMsgToVector(geometry_msgs::Quaternion ros_q){
    // Convertir a quaternion de TF
    tf::Quaternion tf_q;
    tf::quaternionMsgToTF(ros_q, tf_q);
    // Convertir quaternion a vector TF
    tf::Vector3 unitv(1,0,0);
    unitv = tf::quatRotate(tf_q, unitv);
    unitv.normalize();
    // Convertir a vector Eigen
    return Eigen::Vector3f(unitv.x(), unitv.y(), unitv.z());
}
Eigen::Quaternionf Util::eigenVectorToQuaternion(Eigen::Vector3f v){
    Eigen::Quaternionf q;
    q.w() = 0;
    q.vec() = v;
    return q;
}
// Operaciones con nubes de puntos
PointCloud<PointXYZ>::Ptr Util::subsampleCloud(PointCloud<PointXYZ>::Ptr cloud_in, float leafsize){
    PointCloud<PointXYZ>::Ptr cloud_out(new PointCloud<PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> subsampler;
    subsampler.setInputCloud(cloud_in);
    subsampler.setLeafSize(leafsize, leafsize, leafsize);
    subsampler.filter(*cloud_out);
    cloud_out->header.frame_id = cloud_in->header.frame_id;
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
void Util::getClosestPoint(PointCloud<PointXYZ>::Ptr cloud, geometry_msgs::PointStamped &closest_point, float &closest_point_distance){
    KdTree<PointXYZ>::Ptr kdtree (new KdTreeFLANN<PointXYZ>());
    if (cloud->header.frame_id.compare(Util::BASE_FRAME) != 0){
        PointCloud<PointXYZ>::Ptr cloud_base (new PointCloud<PointXYZ>()); 
        // Transformar nube de kinect a base
        tf::TransformListener tf_listener;
        tf::StampedTransform stamped_tf;
        string cloud_frame = cloud->header.frame_id;
        try{
            ROS_INFO("UTIL: Esperando transformación disponible...");
            while (not tf_listener.waitForTransform(cloud_frame, Util::BASE_FRAME, ros::Time(0), ros::Duration(WAIT_TF_TIMEOUT))){
                ROS_ERROR("UTIL: Transformación no pudo ser obtenida en %f segundos",WAIT_TF_TIMEOUT);
                //iterar
            }
            ROS_INFO("UTIL: Transformación obtenida");
            tf_listener.lookupTransform(cloud_frame, Util::BASE_FRAME, ros::Time(0), stamped_tf);
        }
        catch(tf::TransformException ex){
            ROS_ERROR("UTIL: Excepción al obtener transformación: '%s'", ex.what());
        }
        pcl_ros::transformPointCloud(Util::BASE_FRAME, *cloud, *cloud_base, tf_listener);
        kdtree->setInputCloud(cloud_base->makeShared());
    }
    else
        kdtree->setInputCloud(cloud->makeShared());
    vector<int> nearest_index(1);
    vector<float> nearest_dist(1);
    ROS_INFO("UTIL: Buscando el punto más cercano");
    kdtree->nearestKSearch(PointXYZ(0,0,0), 1, nearest_index, nearest_dist);
    ROS_INFO("UTIL: Punto más cercano es (%f, %f, %f) a distancia %fm", cloud->points[nearest_index[0]].x,cloud->points[nearest_index[0]].y, cloud->points[nearest_index[0]].z, nearest_dist[0]);
    geometry_msgs::PointStamped closest_point_base;
    closest_point_base.header.frame_id = Util::BASE_FRAME;
    closest_point_base.point.x = cloud->points[nearest_index[0]].x;
    closest_point_base.point.y = cloud->points[nearest_index[0]].y;
    closest_point_base.point.z = cloud->points[nearest_index[0]].z;
    // Return
    closest_point = closest_point_base;
    closest_point_distance = nearest_dist[0];
}
Eigen::Matrix4f Util::getTransformation(string frame_ini, string frame_end){
    tf::TransformListener tf_listener;
    tf::StampedTransform stamped_tf;
    ros::Time transform_time = ros::Time(0);
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    if (frame_ini.compare(frame_end) == 0)
        return transformation;
    try{
        ROS_INFO("UTIL: Esperando transformacion disponible...");
        if (not tf_listener.waitForTransform(frame_end, frame_ini, transform_time, ros::Duration(WAIT_TF_TIMEOUT))){
            ROS_ERROR("UTIL: Transformacion no pudo ser obtenida antes del timeout (%fs)", WAIT_TF_TIMEOUT);
            return transformation;
        }
        ROS_INFO("UTIL: Guardando transformacion");
        tf_listener.lookupTransform(frame_end, frame_ini, ros::Time(0), stamped_tf);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("UTIL: Excepcion al obtener transformacion: %s", ex.what());
        return transformation;
    }
    tf::Matrix3x3 rotation = stamped_tf.getBasis();
    tf::Vector3 translation = stamped_tf.getOrigin();
    Eigen::Matrix3f upleft3x3;
    upleft3x3 << rotation[0][0], rotation[0][1], rotation[0][2],
                 rotation[1][0], rotation[1][1], rotation[1][2],
                 rotation[2][0], rotation[2][1], rotation[2][2];
    // cout << "Rotation: \n" << upleft3x3 << endl;
    // cout << "Translation: \n" << translation[0] << "    \t" << translation[1] << "    \t" << translation[2] << endl; 
    transformation.block<3,3>(0,0) = upleft3x3;
    transformation.col(3) = Eigen::Vector4f(translation[0], translation[1], translation[2], 1);
    // cout << "Todo: \n" << transformation << endl;
    return transformation;
    /*  transformation << rotation[0][0], rotation[0][1], rotation[0][2], translation[0],
        rotation[1][0], rotation[1][1], rotation[1][2], translation[1],
        rotation[2][0], rotation[2][1], rotation[2][2], translation[2],
        0, 0, 0, 1;*/
}
PolygonMesh Util::getConvexHull(PointCloud<PointXYZ>::Ptr cloud){
    PolygonMesh mesh;
    ConvexHull<PointXYZ> chull;
    chull.setInputCloud(cloud);
    chull.reconstruct(mesh);
    return mesh;
}
PolygonMesh Util::getTriangulation(PointCloud<PointXYZ>::Ptr cloud){
    // Calcular normales
    NormalEstimation<PointXYZ, Normal> n;
    PointCloud<Normal>::Ptr normals (new PointCloud<Normal>());
    search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>());
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(20);
    n.compute(*normals);
    // Crear nube con normales
    PointCloud<PointNormal>::Ptr cloud_with_normals (new PointCloud<PointNormal>());
    concatenateFields(*cloud, *normals, *cloud_with_normals);
    // 
    search::KdTree<PointNormal>::Ptr tree2 (new search::KdTree<PointNormal>());
    tree2->setInputCloud(cloud_with_normals);
    // Triangular
    PolygonMesh triangles;
    GreedyProjectionTriangulation<PointNormal> gp3;
    gp3.setSearchRadius(SEG_THRESHOLD);
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(50);
    gp3.setMaximumSurfaceAngle(Util::PI/4); // 45°
    gp3.setMinimumAngle(Util::PI/18); // 10°
    gp3.setMaximumAngle(2*Util::PI/3); // 120°
    gp3.setNormalConsistency(false);
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(triangles);
    return triangles;
}
// Transformaciones
Eigen::Vector3f Util::transformVector(Eigen::Vector3f vector_in, Eigen::Matrix4f transf){
    Eigen::Vector4f vector_in_4f(vector_in[0], vector_in[1], vector_in[2], 1);
    // Obtener rotación desde matriz de transformación
    Eigen::Matrix4f rot;
    rot = transf;
    rot.col(3) = Eigen::Vector4f(0, 0, 0, 1);
    Eigen::Vector4f vector_out_4f = rot*Eigen::Vector4f(vector_in[0], vector_in[1], vector_in[2], 1);
    Eigen::Vector3f vector_out (vector_out_4f[0], vector_out_4f[1], vector_out_4f[2]);
    return vector_out;
}
geometry_msgs::Point Util::transformPoint(geometry_msgs::Point point_in, Eigen::Matrix4f transf){
    geometry_msgs::Point point_out;
    Eigen::Vector4f point_4f;
    point_4f << point_in.x, point_in.y, point_in.z, 1;
    point_4f = transf*point_4f;
    point_out.x = point_4f[0];
    point_out.y = point_4f[1];
    point_out.z = point_4f[2];
    return point_out;
}
geometry_msgs::Pose Util::transformPose(geometry_msgs::Pose pose_in, Eigen::Matrix4f transf){
    geometry_msgs::Pose pose_out;
    // Obtener rotación desde matriz de transformación
    Eigen::Matrix4f rot;
    rot = transf;
    rot.col(3) = Eigen::Vector4f(0, 0, 0, 1);
    // Crear normal desde quaternion
    tf::Quaternion tf_q;
    tf::quaternionMsgToTF(pose_in.orientation, tf_q);
    tf::Vector3 normal(1, 0, 0);
    normal = tf::quatRotate(tf_q, normal);
    normal.normalize();
    // Rotar normal
    Eigen::Vector3f normal_vector (normal.x(), normal.y(), normal.z());
    Eigen::Vector3f normal_rotated = Util::transformVector(normal_vector, transf);
    normal_rotated.normalize();
    // Obtener quaternion desde normal rotada
    pose_out.orientation = Util::coefsToQuaternionMsg(normal_rotated[0], normal_rotated[1], normal_rotated[2]);
    // Transportar posición
    pose_out.position = Util::transformPoint(pose_in.position, transf);
    return pose_out;
}
Eigen::Matrix3f Util::getRotationBetweenVectors(Eigen::Vector3f vini, Eigen::Vector3f vend){
    Eigen::Vector3f norm_vini = vini.normalized();
    Eigen::Vector3f norm_vend = vend.normalized();
    Eigen::Vector3f v = norm_vini.cross(norm_vend);
    float s = v.norm();
    float c = norm_vini.dot(norm_vend);
    printf("UTIL, getRotation: Angulo entre vectores: %f\n", Util::toGrad(acos(c)));
    Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f vskew, vskew2;
    vskew << 0, -v[2], v[1],
             v[2], 0, -v[0],
             -v[1], v[0], 0;
    vskew2 = vskew*vskew;
    return R + vskew + vskew2*((1-c)/s);
}
Eigen::Quaternionf Util::getQuaternionBetweenVectors(Eigen::Vector3f vini, Eigen::Vector3f vend){
    vini.normalize();
    vend.normalize();
    Eigen::Vector3f cross = vini.cross(vend);
    Eigen::Quaternionf rotation_q;
    rotation_q.x() = cross.x();
    rotation_q.y() = cross.y();
    rotation_q.z() = cross.z();
    rotation_q.w() = 1 + vini.dot(vend); // Asume que vini y vend están normalizados
    return rotation_q;
}
// Utilidades específicas Placing
/**
 * searchPlacingSurface: Función que busca iterativamente en una nube de puntos la ocurrencia de una superficie plana sensata para efectuar el placing.
 *                         Este método debe ser llamado INMEDIATAMENTE luego de obtener la nube de puntos desde el kinect.
 * @param  cloud_in          : Nube de puntos donde buscar
 * @param  &cloud_out        : Nube de puntos que representa a la superficie RELATIVA A ODOM
 * @param  &surface_normal   : Normal de la superficie RELATIVA A ODOM
 * @param  &surface_centroid : Centroide nube RELATIVO A ODOM
 * @param  min_height        : Mínima altura aceptable segun ODOM
 * @param  max_height        : Máxima altura aceptable segun ODOM
 * @param  inclination       : Máxima inclinación aceptable, en radianes, relativa al plano XY. Mesa lisa tiene inclinación PI/2
 * @return  True en éxito, False en caso contrario.
 */
bool Util::searchPlacingSurface(PointCloud<PointXYZ>::Ptr cloud_in, PointCloud<PointXYZ>::Ptr &cloud_out, geometry_msgs::PoseStamped &surface_normal, geometry_msgs::PointStamped &surface_centroid, float min_height, float max_height, float inclination){
    // variables auxiliares, para borrar
    ros::NodeHandle nh;
    // ros::Publisher point_pub1 = nh.advertise<geometry_msgs::PointStamped>("centroide_k", 1);
    ros::Publisher point_pub2 = nh.advertise<geometry_msgs::PointStamped>("surface_centroid", 1);
    ros::Publisher surface_normal_pub = nh.advertise<geometry_msgs::PoseStamped>("surface_normal", 1);
    // ros::Publisher pc_pub1 = nh.advertise<PointCloud<PointXYZ> >("plano", 1);
    // ros::Publisher pc_pub2 = nh.advertise<PointCloud<PointXYZ> >("subsampled", 1);
    // pc_pub2.publish(*cloud_in);
    // Tamaño inicial de la nube de entrada
    int init_cloud_size = (int)cloud_in->size();
    int current_cloud_size = init_cloud_size;
    Eigen::Matrix4f transformation = Util::getTransformation(cloud_in->header.frame_id, Util::ODOM_FRAME);
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
                ROS_INFO("UTIL: No se encontró plano");
                return false;
            }
            // Si es una superficie muy chica
            ROS_INFO("UTIL: Superficie muy pequeña (%f%% de un mímino de %f%% del total de puntos)", inliers->indices.size()*100.0/init_cloud_size, MIN_SEGMENTED_SURFACE_PERCENT);
            return false;
        }
        ROS_INFO("UTIL: Se obtuvo un modelo con %i puntos", (int)inliers->indices.size());
        extractor.setInputCloud(cloud_in);
        extractor.setIndices(inliers);
        extractor.setNegative(false);
        extractor.filter(*cloud_plane);
        // Verificar primer criterio de aceptación: Altura mínima
        ROS_INFO("UTIL: Area aproximada de la superficie: %fm2", Util::SUBSAMPLE_LEAFSIZE*Util::SUBSAMPLE_LEAFSIZE*(int)inliers->indices.size());
        geometry_msgs::PointStamped cloud_centroid, cloud_centroid_base;
        cloud_centroid.point = Util::getCloudCentroid(cloud_plane);
        cloud_centroid.header.frame_id = cloud_in->header.frame_id;
        cloud_centroid_base.header.frame_id = Util::ODOM_FRAME;
        cloud_centroid_base.point = Util::transformPoint(cloud_centroid.point, transformation);
        ROS_INFO("UTIL: Centroide según base: (%f, %f, %f)", cloud_centroid_base.point.x, cloud_centroid_base.point.y, cloud_centroid_base.point.z);
        point_pub2.publish(cloud_centroid_base);
        if (cloud_centroid_base.point.z > min_height and cloud_centroid_base.point.z < max_height){
            // Primera prueba superada. Ahora, comprobar inclinación
            // Transformar normal del plano a odom frame. Para eso, tomar normal y rotarla, NO trasladarla
            Eigen::Vector3f normal_kinect (coefs->values[0], coefs->values[1], coefs->values[2]);
            normal_kinect.normalize(); // redundante pero por si acaso.
            Eigen::Vector3f normal_odom = Util::transformVector(normal_kinect, transformation);
            // Corregir orientación para que apunte hacia arriba
            if (normal_odom[2]<0)
                normal_odom *= -1;
            // Obtener ángulo de inclinación de la normal, respecto al plano XY
            tf::Quaternion tf_q;
            tf::quaternionMsgToTF(Util::coefsToQuaternionMsg(normal_odom[0], normal_odom[1], normal_odom[2]), tf_q);
            double roll, pitch, yaw;
            tf::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
            pitch = -abs(pitch);
            ROS_INFO("UTIL: Inclinación del plano respecto al suelo (plano XY): %f grados", Util::toGrad(-pitch));
            // Verificar segundo criterio de aceptación: Inclinación máxima
            if (inclination - PITCH_THRESHOLD < pitch and pitch < inclination + PITCH_THRESHOLD){
                ROS_INFO("UTIL: Superficie encontrada");
                // Guardar superficie encontrada
                PointCloud<PointXYZ>::Ptr cloud_out_pre (new PointCloud<PointXYZ>());
                extractor.setInputCloud(cloud_in);
                extractor.setIndices(inliers);
                extractor.setNegative(false);
                extractor.filter(*cloud_out_pre);
                // Transformar de frame kinect a odom frame
                transformPointCloud(*cloud_out_pre, *cloud_out, transformation);
                cloud_out->header.frame_id = Util::ODOM_FRAME;
                // Guardar normal de la superficie, centrada en centroide y respecto a odom.
                geometry_msgs::PoseStamped pose_normal;
                pose_normal.header.frame_id = Util::ODOM_FRAME;
                pose_normal.pose.position = cloud_centroid_base.point;
                tf::quaternionTFToMsg(tf_q, pose_normal.pose.orientation);
                surface_normal_pub.publish(pose_normal);
                surface_normal_pub.publish(pose_normal);
                surface_normal_pub.publish(pose_normal);
                surface_normal = pose_normal;
                surface_centroid = cloud_centroid_base;
                return true;
            }
        }
        else {
            ROS_INFO("UTIL: Plano demasiado %s. Buscando siguiente...", cloud_centroid_base.point.z > max_height ? "alto" : "bajo");
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
bool Util::gripperFilter(PointCloud<PointXYZ>::Ptr cloud_in, PointCloud<PointXYZ>::Ptr &object_out, PointCloud<PointXYZ>::Ptr &gripper_out){
    ROS_INFO("UTIL: Comienza filtro del gripper");
    if (cloud_in->header.frame_id.find(Util::GRIPPER_FRAME_SUFFIX) == string::npos){
        ROS_ERROR("UTIL: No se puede filtrar gripper: Frame de nube es incorrecto");
        ROS_ERROR("UTIL: Frame debe contener '%s' pero era '%s'", Util::GRIPPER_FRAME_SUFFIX.c_str(), cloud_in->header.frame_id.c_str());
        return false;
    }
    ROS_INFO("UTIL: Agregando boxes al filtro");
    vector<Box> gripper_boxes;
    // inicializar boxes. Total y absolutamente HARDCODEADO, basado en observaciones.
    Box box1;
    box1.center[0] = -0.095; box1.center[1] = box1.center[2] = 0;
    box1.size[0] = 0.098; box1.size[1] = 0.16; box1.size[2] = 0.061;
    gripper_boxes.push_back(box1);
    // Dedos del gripper
    Box box2;
    box2.center[0] = -0.03; box2.center[1] = box2.center[2] = 0;
    box2.size[0] = 0.105; box2.size[1] = 0.18; box2.size[2] = 0.03;
    gripper_boxes.push_back(box2);
    // Pedazo cuando está cerrado
    Box box3;
    box3.center[0] = -0.042; box3.center[1] = box3.center[2] = 0;
    box3.size[0] = 0.02; box3.size[1] = 0.11; box3.size[2] = 0.052;
    gripper_boxes.push_back(box3);

    ROS_INFO("UTIL: Borrando brazo");
    PointCloud<PointXYZ>::Ptr cloud_out (new PointCloud<PointXYZ>());
    // Borrar primer pedazo
    PassThrough<PointXYZ> pass;
    pass.setInputCloud(cloud_in);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-0.14, 1000); // Totalmente HARDCODEADO
    pass.filter(*cloud_out);
    // Separar gripper del objeto
    /*    PointCloud<PointXYZ>::Ptr object_pc(new PointCloud<PointXYZ>()),
                              gripper_pc(new PointCloud<PointXYZ>());*/
    // Capturar índices de puntos dentro de paralelepípedo
    ROS_INFO("UTIL: Separando gripper del objeto según boxes");
    PointIndices::Ptr inliers (new PointIndices());
    for (int i=0; i < cloud_out->points.size(); i++){
        for (int j=0; j < gripper_boxes.size(); j++){
            if (isPointInsideBox(cloud_out->points[i], gripper_boxes[j])){
                inliers->indices.push_back(i);
                break;
            }
        }
    }
    // Extraer índices de cada figura
    ExtractIndices<PointXYZ> extractor;
    extractor.setInputCloud(cloud_out);
    extractor.setIndices(inliers);
    extractor.setNegative(false);
    extractor.filter(*gripper_out);
    extractor.setNegative(true);
    extractor.filter(*object_out);
    return true;
}
PointIndices::Ptr Util::getFactiblePlacingPointsIndices(PointCloud<PointXYZ>::Ptr surface_in, geometry_msgs::PoseStamped surface_normal_pose, float base_area){
    ros::NodeHandle nh_;
    ros::Publisher test_proj_pub;
    ros::Publisher test_chull_pub;
    test_chull_pub = nh_.advertise<PointCloud<PointXYZ> >("chull", 1);
    test_proj_pub = nh_.advertise<PointCloud<PointXYZ> >("flat_proj", 1);
    ros::Duration(1.0).sleep();
    // Preprocesamiento de nube de puntos de la superficie.
    //      En este punto se quiere descartar los más puntos posibles que signifiquen poses
    //      peligrosas, tanto del borde de la superficie como cercanas a otros objetos.
    
    // Quitar puntos de borde. 
    //      Algoritmo: - Obtener concave hull del plano (representa el borde de la superficie)
    //                 - Descartar todos los puntos cercanos a algún punto del concave hull
    
    // Congregar puntos de nube en objeto de indices
    PointIndices::Ptr inliers (new PointIndices());
    inliers->indices.resize((int)surface_in->points.size());
    for (int i=0; i<(int)inliers->indices.size(); i++)
        inliers->indices[i] = i;
    printf("UTIL: Nro puntos de superficie: %d\n", (int)surface_in->points.size());
    // Obtener coeficientes del plano a partir de la normal
    Eigen::Vector3f normal_orientation_vector = Util::quaternionMsgToVector(surface_normal_pose.pose.orientation);
    ModelCoefficients::Ptr surface_coefs (new ModelCoefficients());
    surface_coefs->values.resize(4);
    surface_coefs->values[0] = normal_orientation_vector[0];
    surface_coefs->values[1] = normal_orientation_vector[1];
    surface_coefs->values[2] = normal_orientation_vector[2];
    surface_coefs->values[3] = -surface_normal_pose.pose.position.z;
    // proyectar puntos sobre su modelo representado
    PointCloud<PointXYZ>::Ptr surface_projected (new PointCloud<PointXYZ>());
    ProjectInliers<PointXYZ> proj;
    proj.setModelType(SACMODEL_PLANE);
    proj.setIndices(inliers);
    proj.setInputCloud(surface_in);
    proj.setModelCoefficients(surface_coefs);
    proj.filter(*surface_projected);
    test_proj_pub.publish(surface_projected);
    test_proj_pub.publish(surface_projected);
    test_proj_pub.publish(surface_projected);
    printf("Place: Nro puntos de proyeccion: %d\n", (int)surface_projected->points.size());
    // Obtener concave hull
    PointCloud<PointXYZ>::Ptr cloud_hull (new PointCloud<PointXYZ>());
    ConcaveHull<PointXYZ> chull;
    chull.setInputCloud(surface_projected);
    chull.setAlpha(0.1);
    chull.reconstruct(*cloud_hull);
    printf("Place: Nro puntos chull: %d\n", (int)cloud_hull->points.size());
    test_chull_pub.publish(cloud_hull);
    test_chull_pub.publish(cloud_hull);
    test_chull_pub.publish(cloud_hull);

    return inliers;
}

bool Util::isPointCloudCutByPlane(PointCloud<PointXYZ>::Ptr cloud, ModelCoefficients::Ptr coefs, PointXYZ p_plane){
    /*
    Algoritmo:
        - Obtener vector normal a partir de coeficientes
        - Iterar sobre puntos de la nube
            - Calcular vector delta entre punto entregado (dentro del plano) y punto iterado
            - Calcular ángulo entre vector delta y normal
            - Angulos menores a PI/2 son de un lado, mayores son de otro
            - Si aparecen puntos de ambos lados, retornar True, else, false.
    */
    Eigen::Vector3f normal = Eigen::Vector3f(coefs->values[0], coefs->values[1], coefs->values[2]);
    // normal.normalize();
    // cout << "Normal: " << normal << endl;
    bool side;
    // Iterar sobre los puntos
    for (int i=0; i<cloud->points.size(); i++){
        // Calcular ángulo entre punto y normal
        Eigen::Vector3f delta = Eigen::Vector3f (p_plane.x, p_plane.y, p_plane.z) - Eigen::Vector3f(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
        // delta.normalize();
        // double alpha = acos(normal.dot(delta));
        float alpha = Util::angleBetweenVectors(normal[0], normal[1], normal[2], delta[0], delta[1], delta[2]);
        // printf ("Alpha: %f°\n", (alpha*180/PI));
        if (i==0){
            side = (alpha < Util::PI/2.0);
            // printf("Lado escogido: %s", side ? "true": "false");
            continue;
        }
        if (side != (alpha < Util::PI/2.0)){
            // printf("Nube es cortada por plano\n");
            return true;
        }
    }
    // printf("Nube NO es cortada por plano\n");
    return false;
}
// Collision objects
void worldCallback(const moveit_msgs::PlanningScene::Ptr &scene){
    if (not get_world)
        return;
    // Llegó un mensaje del world. Guardar lista de objetos
    ROS_INFO("UTIL: Recibiendo update del world");
    objects.clear();
    for (int i = 0; i<(int)(scene->world.collision_objects.size()) ; i++){
        objects.push_back(scene->world.collision_objects[i]);
    }
    attached_objects.clear();
    for (int i = 0; i < (int)(scene->robot_state.attached_collision_objects.size()) ; i++){
        attached_objects.push_back(scene->robot_state.attached_collision_objects[i]);
    }
    get_world = false;
}
/**
 * enableDefaultGripperCollisions habilita o deshabilita la esfera de protección del gripper. Permite que se mueva libremente sin chocar con el objeto tomado.
 * @param attached_object_pub  Publicador para objetos attachados al robot
 * @param collision_object_pub Publicador para collision objects del world
 * @param enable               Habilitar o deshabilitar
 * @param which_gripper        Cual de los dos grippers
 * @param radius               Tamaño de la bola. Tiene valor default.
 */
void Util::enableDefaultGripperCollisions(ros::Publisher &attached_object_pub, ros::Publisher &collision_object_pub, bool enable, char which_gripper){
    // Crear collision object
    moveit_msgs::CollisionObject co;
    co.id = Util::COLLISION_BALL_ID;
    moveit_msgs::AttachedCollisionObject a_co;
    // a_co.link_name = (which_gripper == 'l' ? "l_wrist_roll_link" : "r_wrist_roll_link");
    a_co.link_name = (which_gripper == 'l' ? "l" : "r") + Util::GRIPPER_LINK_PREFIX;
    // co.header.frame_id = (which_gripper == 'l' ? "l_gripper_tool_frame" : "r_gripper_tool_frame");
    co.header.frame_id = (which_gripper == 'l' ? "l" : "r") + Util::GRIPPER_FRAME_SUFFIX;
    // Crear shape y su pose, y añadirla
    shape_msgs::SolidPrimitive shape;
    shape.type = shape.SPHERE;
    const float shape_size = Util::COLLISION_BALL_RADIUS;
    shape.dimensions.push_back(shape_size); // x
    shape.dimensions.push_back(shape_size); // y
    shape.dimensions.push_back(shape_size); // z 
    geometry_msgs::Pose co_pose;
    co_pose.orientation.w = 1;
    co.primitives.push_back(shape);
    co.primitive_poses.push_back(co_pose);
    // Configurar attached para agregar a gripper
    // Agregando links permitidos
    Util::setAllowedCollisionLinks(a_co, which_gripper);
    // Añadiendo objeto
    if (enable){
        co.operation = co.ADD;
    }
    else{
        co.operation = co.REMOVE;
    }
    a_co.object = co;
    // Publicar nuevo objeto attachado al robot
    attached_object_pub.publish(a_co);
    if (enable){
        ROS_INFO("UTIL: Haciendo tiempo para que desaparezcan voxels dentro de collision object...");
        ros::Duration(Util::VOXEL_UPDATE_DELAY).sleep(); // Para esperar que desaparezcan voxels dentro de la esfera;
        return;
    }
    ros::Duration(0.3).sleep(); // Para asegurarse de que el tópico escuchó nuestra petición y la entendió, por ende, desattachó.
    // En este punto, el objeto está desattachado del robot pero existe en el mundo.
    // Hay que eliminarlo ahora del mundo de manera segura.
    Util::removeCollisionObjectFromWorld(collision_object_pub, co);
}
/**
 * attachBoundingBoxToGripper Añade bounding box del objeto graspeado al gripper, para checkear colisiones al momento de planear con moveit
 * @param attached_object_pub  Publicador para objetos attachados al robot
 * @param collision_object_pub Publicador para collision objects del world
 * @param which_gripper        Cual de los dos grippers
 * @param bounding_box         Bounding box representado por struct personalizado
 */
void Util::attachBoundingBoxToGripper(ros::Publisher &attached_object_pub, ros::Publisher &collision_object_pub, char which_gripper, BBOriented bounding_box){
    // Crear collision object
    moveit_msgs::CollisionObject co;
    co.id = "object_bounding_box";
    moveit_msgs::AttachedCollisionObject a_co;
    a_co.link_name = (which_gripper == 'l' ? "l_wrist_roll_link" : "r_wrist_roll_link");
    co.operation = co.ADD;
    co.header.frame_id = (which_gripper == 'l' ? "l_gripper_tool_frame" : "r_gripper_tool_frame");
    // Crear shape y su pose, y añadirla
    shape_msgs::SolidPrimitive shape;
    shape.type = shape.BOX;
    shape.dimensions.push_back(abs(bounding_box.min.x - bounding_box.max.x)); // x
    shape.dimensions.push_back(abs(bounding_box.min.y - bounding_box.max.y)); // y
    shape.dimensions.push_back(abs(bounding_box.min.z - bounding_box.max.z)); // z
    geometry_msgs::Pose co_pose;
    co_pose.position = bounding_box.position;
    co_pose.orientation = bounding_box.rotation;
    co.primitives.push_back(shape);
    co.primitive_poses.push_back(co_pose);
    // Configurar attached para agregar a gripper
    // Agregando links permitidos
    Util::setAllowedCollisionLinks(a_co, which_gripper);
    // Añadiendo objeto
    a_co.object = co;
    // Publicar nuevo objeto attachado al robot
    attached_object_pub.publish(a_co);
}
/**
 * detachBoundingBoxFromGripper elimina bounding box del objeto graspeado del gripper y luego del mundo.
 * @param attached_object_pub  Publicador para objetos attachados al robot
 * @param collision_object_pub Publicador para collision objects del world
 * @param which_gripper        Cual de los dos grippers
 */
void Util::detachBoundingBoxFromGripper(ros::Publisher &attached_object_pub, ros::Publisher &collision_object_pub, char which_gripper){
    moveit_msgs::CollisionObject co;
    co.id = "object_bounding_box";
    moveit_msgs::AttachedCollisionObject a_co;
    a_co.link_name = (which_gripper == 'l' ? "l_wrist_roll_link" : "r_wrist_roll_link");
    // Publicar dettach del robot
    attached_object_pub.publish(a_co);
    ros::Duration(0.3).sleep();
    // En este punto, el objeto está desattachado del robot pero existe en el mundo.
    // Hay que eliminarlo ahora del mundo de manera segura.
    Util::removeCollisionObjectFromWorld(collision_object_pub, co);
}
/**
 * attachMeshToGripper es similar a attachBoundingBoxToGripper, pero en vez de usar un bounding box, usa el mesh, para dibujar de forma más exacta el collision object.
 * @param attached_object_pub  Publicador para attached_collision_object
 * @param collision_object_pub Publicador para collision_object
 * @param object_mesh          Malla del objeto, representada como Polymesh
 */
void Util::attachMeshToGripper(ros::Publisher &attached_object_pub, ros::Publisher &collision_object_pub, char which_gripper, Polymesh &object_mesh){
    moveit_msgs::CollisionObject co;
    co.header.frame_id = (which_gripper == 'l' ? "l_gripper_tool_frame" : "r_gripper_tool_frame");
    co.id = Util::COLLISION_MESH_ID;
    // crear y agregar malla
    shape_msgs::Mesh co_mesh;
    pclPolygonMeshToShapeMsg(object_mesh.getPCLMesh(), co_mesh);
    geometry_msgs::Pose mesh_pose;
    mesh_pose.orientation.w = 1;
    co.meshes.push_back(co_mesh);
    co.mesh_poses.push_back(mesh_pose);
    co.operation = co.ADD;

    // Crear attached collision object
    moveit_msgs::AttachedCollisionObject a_co;
    a_co.link_name = (which_gripper == 'l' ? "l_wrist_roll_link" : "r_wrist_roll_link");
    Util::setAllowedCollisionLinks(a_co, which_gripper);
    // Añadiendo objeto
    a_co.object = co;
    // Publicar
    attached_object_pub.publish(a_co);
    ROS_INFO("UTIL: Haciendo tiempo para que desaparezcan voxels dentro de collision object...");
    ros::Duration(Util::VOXEL_UPDATE_DELAY).sleep(); // Para esperar que desaparezcan voxels dentro de la esfera;
}
void Util::detachMeshFromGripper(ros::Publisher &attached_object_pub, ros::Publisher &collision_object_pub){
    moveit_msgs::CollisionObject co;
    co.id = Util::COLLISION_MESH_ID;
    co.operation = co.REMOVE;
    moveit_msgs::AttachedCollisionObject a_co;
    a_co.object = co;
    attached_object_pub.publish(a_co);
    removeCollisionObjectFromWorld(collision_object_pub, co);
}
void Util::addSurfaceAsCollisionObject(ros::Publisher &collision_object_pub, PolygonMesh &surface_mesh){
    moveit_msgs::CollisionObject co;
    co.header.frame_id = Util::ODOM_FRAME;
    co.id = Util::COLLISION_SURFACE_ID;
    shape_msgs::Mesh co_mesh;
    pclPolygonMeshToShapeMsg(surface_mesh, co_mesh);
    geometry_msgs::Pose mesh_pose;
    mesh_pose.orientation.w = 1;
    co.meshes.push_back(co_mesh);
    co.mesh_poses.push_back(mesh_pose);
    co.operation = co.ADD;
    collision_object_pub.publish(co);
    ROS_INFO("UTIL: Haciendi tiempo para que desaparezcan voxels cerca de la superficie...");
    ros::Duration(Util::VOXEL_UPDATE_DELAY).sleep();
}
void Util::pclPolygonMeshToShapeMsg(PolygonMesh pclmesh, shape_msgs::Mesh &shapemsg){
    // Un mesh se compone de:
    //  1) Una lista de triángulos compuestos de
    //      1.1) Una lista de índices al arreglo de vértices
    //  2) Lista de vértices definidos como:
    //      2.1) Tres coordenadas x y z
    
    /* Algoritmo:
        - Crear vector de vértices a partir de vértices de nube de polygonmesh
        - Crear vector de triángulos a partir de triángulos de polygonmesh
    */
    
    // Obtener malla y nube
    // PolygonMesh pclmesh = pmesh.getPCLMesh();
    // PointCloud<PointXYZ>::Ptr pclcloud = pmesh.getPointCloud();
    // int n_triangles = pmesh.getPolygonNumber();
    PointCloud<PointXYZ>::Ptr pclcloud (new PointCloud<PointXYZ>());
    fromPCLPointCloud2(pclmesh.cloud, *pclcloud);
    int n_triangles = (int)pclmesh.polygons.size();
    int n_vertices = (int)pclcloud->points.size();
    // Poblar vértices
    for (int i = 0; i < n_vertices; i++){
        // Los índices de los vértices nuevos se mapean 1:1 a los índices del pointcloud original
        geometry_msgs::Point new_vertice;
        new_vertice.x = pclcloud->points[i].x;
        new_vertice.y = pclcloud->points[i].y;
        new_vertice.z = pclcloud->points[i].z;
        shapemsg.vertices.push_back(new_vertice);  
    }
    // Poblar triángulos
    for (int i = 0; i < n_triangles; i++){
        shape_msgs::MeshTriangle new_triangle;
        new_triangle.vertex_indices[0] = pclmesh.polygons[i].vertices[0];
        new_triangle.vertex_indices[1] = pclmesh.polygons[i].vertices[1];
        new_triangle.vertex_indices[2] = pclmesh.polygons[i].vertices[2];
        shapemsg.triangles.push_back(new_triangle);
    }
    // listo
}
// PRIVATE
bool Util::isPointInsideBox(PointXYZ p, Box box){
    bool in_x = (p.x > box.center[0] - box.size[0]/2.0) and (p.x < box.center[0] + box.size[0]/2.0);
    bool in_y = (p.y > box.center[1] - box.size[1]/2.0) and (p.y < box.center[1] + box.size[1]/2.0);
    bool in_z = (p.z > box.center[2] - box.size[2]/2.0) and (p.z < box.center[2] + box.size[2]/2.0);
    return in_x and in_y and in_z;
}
void Util::setAllowedCollisionLinks(moveit_msgs::AttachedCollisionObject &a_co, char which_gripper){
    a_co.touch_links.push_back(which_gripper == 'l' ? "l_gripper_r_finger_tip_link" :           "r_gripper_r_finger_tip_link"); 
    a_co.touch_links.push_back(which_gripper == 'l' ? "l_gripper_r_finger_link" :               "r_gripper_r_finger_link"); 
    a_co.touch_links.push_back(which_gripper == 'l' ? "l_gripper_l_finger_tip_link" :           "r_gripper_l_finger_tip_link"); 
    a_co.touch_links.push_back(which_gripper == 'l' ? "l_gripper_l_finger_link" :               "r_gripper_l_finger_link"); 
    a_co.touch_links.push_back(which_gripper == 'l' ? "l_wrist_roll_link" :                     "r_wrist_roll_link"); // redundante
    a_co.touch_links.push_back(which_gripper == 'l' ? "l_wrist_flex_link" :                     "r_wrist_flex_link");
    a_co.touch_links.push_back(which_gripper == 'l' ? "l_forearm_roll_link" :                   "r_forearm_roll_link");
    a_co.touch_links.push_back(which_gripper == 'l' ? "l_elbow_flex_link" :                     "r_elbow_flex_link");
    a_co.touch_links.push_back(which_gripper == 'l' ? "l_upper_arm_roll_link" :                 "r_upper_arm_roll_link");
    a_co.touch_links.push_back(which_gripper == 'l' ? "l_shoulder_lift_link" :                  "r_shoulder_lift_link");
    a_co.touch_links.push_back(which_gripper == 'l' ? "l_forearm_link" :                        "r_forearm_link");
    a_co.touch_links.push_back(which_gripper == 'l' ? "l_gripper_motor_accelerometer_link" :    "r_gripper_motor_accelerometer_link");
    a_co.touch_links.push_back(which_gripper == 'l' ? "l_gripper_palm_link" :                   "r_gripper_palm_link");
    a_co.touch_links.push_back(which_gripper == 'l' ? "l_upper_arm_link"  :                     "r_upper_arm_link");
    a_co.touch_links.push_back("base_link");
}
void Util::removeCollisionObjectFromWorld(ros::Publisher &collision_object_pub, moveit_msgs::CollisionObject co){
    // Subscribir al tópico que informa del mundo para verificar que se borró el objeto
    ros::NodeHandle nh_;
    ros::Subscriber world_sub = nh_.subscribe("/move_group/monitored_planning_scene", 1, worldCallback);
    ros::Duration(0.5).sleep();
    co.operation = co.REMOVE;
    while (1){
        // Eliminarlo del collision world
        ROS_INFO("UTIL: Borrando del world");
        collision_object_pub.publish(co);
        // Esperar actualización del estado del world
        get_world = true;
        while (get_world){
            ros::Duration(0.5).sleep();
        }
        // En este punto, ya recibí una actualización de los collision objects del world
        // si está vacío, ok
        if ((int)objects.size() == 0){
            break;
        }
        // Si no, revisar que no se encuentre el objeto en cuestión
        bool restart_loop = false;
        for (int i = 0; i < (int)objects.size(); i++){
            if (objects[i].id == co.id){
                // Reintentar
                ROS_WARN("UTIL: '%s' todavía no desaparece del world. Reintentando...", co.id.c_str());
                restart_loop = true;
                break;
            }
        }
        if (restart_loop)
            continue;
        // ok, no está
        break;
    }
    ROS_INFO("UTIL: '%s' correctamente eliminado del mundo", co.id.c_str());  
}
