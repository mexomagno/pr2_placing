#include "../Util.h"

// PUBLIC
// CONSTANTES
// Frames
const string Util::BASE_FRAME                  = "/base_footprint";
const string Util::ODOM_FRAME                  = "/odom_combined";
const string Util::CAMERA_FRAME                = "/high_def_frame";
const string Util::KINECT_FRAME                = "/head_mount_kinect_rgb_optical_frame";

// Tópicos
const string Util::KINECT_TOPIC                = "/head_mount_kinect/depth_registered/points";
const string Util::KINECT_TOPIC_SELF_FILTERED  = "/head_mount_kinect/depth_registered/points";
const string Util::GRIPPER_GOAL_TOPIC_SUFFIX   = "_gripper_controller/gripper_action/goal";
const string Util::GRIPPER_STATUS_TOPIC_SUFFIX = "_gripper_controller/gripper_action/result";
const string Util::BASE_CONTROLLER_TOPIC       = "/base_controller/command";
const string Util::ODOM_TOPIC                  = "/base_odometry/odom";

// Otros
const float Util::PI                      = 3.1416;
const float Util::SUBSAMPLE_LEAFSIZE      = 0.05;
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
// Gripper Scanner
const float Util::SCAN_ROLL_DELTA         = Util::PI/2.0;
const string Util::GRIPPER_FRAME_SUFFIX   = "_gripper_tool_frame";
const float Util::scan_position[]         = {0.6, 0, 1.2};
const float Util::scan_orientation[]      = {0, -Util::PI/2.0, 0}; //R, P, Y
const float Util::GRIPPER_STABILIZE_TIME  = 1;
const float Util::SCAN_PASSTHROUGH_Z      = 0.5;
const float Util::SCAN_LEAFSIZE           = 0.005;
// Stable surface
const double PATCH_ANGLE_THRESHOLD        = 0.2;

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
            ROS_DEBUG("UTIL: Esperando transformación disponible...");
            while (not tf_listener.waitForTransform(cloud_frame, Util::BASE_FRAME, ros::Time(0), ros::Duration(WAIT_TF_TIMEOUT))){
                ROS_ERROR("UTIL: Transformación no pudo ser obtenida en %f segundos",WAIT_TF_TIMEOUT);
                //iterar
            }
            ROS_DEBUG("UTIL: Transformación obtenida");
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
    ROS_DEBUG("UTIL: Buscando el punto más cercano");
    kdtree->nearestKSearch(PointXYZ(0,0,0), 1, nearest_index, nearest_dist);
    ROS_DEBUG("UTIL: Punto más cercano es (%f, %f, %f) a distancia %fm", cloud->points[nearest_index[0]].x,cloud->points[nearest_index[0]].y, cloud->points[nearest_index[0]].z, nearest_dist[0]);
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
        ROS_DEBUG("UTIL: Esperando transformacion disponible...");
        if (not tf_listener.waitForTransform(frame_end, frame_ini, transform_time, ros::Duration(WAIT_TF_TIMEOUT))){
            ROS_ERROR("UTIL: Transformacion no pudo ser obtenida antes del timeout (%fs)", WAIT_TF_TIMEOUT);
            return transformation;
        }
        ROS_DEBUG("UTIL: Guardando transformacion");
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
// Utilidades específicas
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
    // variables auxiliares, para borrar
    ros::NodeHandle nh;
    ros::Publisher point_pub1 = nh.advertise<geometry_msgs::PointStamped>("centroide_k", 1);
    ros::Publisher point_pub2 = nh.advertise<geometry_msgs::PointStamped>("centroide_b", 1);
    ros::Publisher pc_pub1 = nh.advertise<PointCloud<PointXYZ> >("plano", 1);
    ros::Publisher pc_pub2 = nh.advertise<PointCloud<PointXYZ> >("subsampled", 1);
    pc_pub2.publish(*cloud_in);
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
        ROS_DEBUG("UTIL: Se obtuvo un modelo con %i puntos", (int)inliers->indices.size());
        extractor.setInputCloud(cloud_in);
        extractor.setIndices(inliers);
        extractor.setNegative(false);
        extractor.filter(*cloud_plane);
        // Verificar primer criterio de aceptación: Altura mínima
        pc_pub1.publish(*cloud_plane);
        ROS_DEBUG("UTIL: Area aproximada de la superficie: %fm2", Util::SUBSAMPLE_LEAFSIZE*Util::SUBSAMPLE_LEAFSIZE*(int)inliers->indices.size());
        geometry_msgs::PointStamped cloud_centroid, cloud_centroid_base;
        cloud_centroid.point = Util::getCloudCentroid(cloud_plane);
        cloud_centroid.header.frame_id = cloud_in->header.frame_id;
        // cloud_centroid.header.stamp = stamped_tf.stamp_;
        point_pub1.publish(cloud_centroid);
        Eigen::Vector4f centroid_vector (cloud_centroid.point.x, cloud_centroid.point.y, cloud_centroid.point.z, 1);
        Eigen::Vector4f centroid_vector_base = transformation*centroid_vector;
        cloud_centroid_base.point.x = centroid_vector_base[0];
        cloud_centroid_base.point.y = centroid_vector_base[1];
        cloud_centroid_base.point.z = centroid_vector_base[2];
        cloud_centroid_base.header.frame_id = Util::ODOM_FRAME;
        // tf_listener.transformPoint(Util::ODOM_FRAME, cloud_centroid, cloud_centroid_base);
        ROS_DEBUG("UTIL: Centroide según base: (%f, %f, %f)", cloud_centroid_base.point.x, cloud_centroid_base.point.y, cloud_centroid_base.point.z);
        point_pub2.publish(cloud_centroid_base);
        if (cloud_centroid_base.point.z > min_height and cloud_centroid_base.point.z < max_height){
            // Primera prueba superada. Ahora, comprobar inclinación
            // Transformar normal del plano a odom frame. Para eso, tomar normal y rotarla, NO trasladarla
            Eigen::Vector4f normal_kinect, normal_odom;
            normal_kinect << coefs->values[0], coefs->values[1], coefs->values[2], 1;
            normal_kinect.normalize();
            Eigen::Matrix4f rotation;
            rotation = transformation;
            // Elimino componente de traslación y la aplico
            rotation.col(3) = Eigen::Vector4f(0, 0, 0, 1);
            normal_odom = rotation*normal_kinect;
            // Obtener ángulo de inclinación de la normal, respecto al plano XY
            tf::Quaternion tf_q;
            tf::quaternionMsgToTF(Util::coefsToQuaternionMsg(normal_odom[0], normal_odom[1], normal_odom[2]), tf_q);
            double roll, pitch, yaw;
            tf::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
            // El siguiente arreglo, puede ser un problema si se encuentra un techo bajo
            pitch = -abs(pitch);
            ROS_DEBUG("UTIL: Inclinación del plano respecto al suelo (plano XY): %f grados", Util::toGrad(-pitch));
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
    ROS_DEBUG("UTIL: Comienza filtro del gripper");
    if (cloud_in->header.frame_id.find(Util::GRIPPER_FRAME_SUFFIX) == string::npos){
        ROS_ERROR("UTIL: No se puede filtrar gripper: Frame de nube es incorrecto");
        ROS_ERROR("UTIL: Frame debe contener '%s' pero era '%s'", Util::GRIPPER_FRAME_SUFFIX.c_str(), cloud_in->header.frame_id.c_str());
        return false;
    }
    ROS_DEBUG("UTIL: Agregando boxes al filtro");
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

    ROS_DEBUG("UTIL: Borrando brazo");
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
    ROS_DEBUG("UTIL: Separando gripper del objeto según boxes");
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
bool Util::getStablePose(PointCloud<PointXYZ>::Ptr object_pc, PointCloud<PointXYZ>::Ptr gripper_pc, geometry_msgs::PoseStamped &pose_out){
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
    mesh.getFlatPatches(PATCH_ANGLE_THRESHOLD, patches, patches_areas);
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
            ROS_INFO("UTIL: Se ha encontrado un plano estable (de area %.2f)\n", best_patch_area);
            // Obtener transformación
            Eigen::Matrix4f transformation = Util::getTransformation(object_pc->header.frame_id, Util::BASE_FRAME);
            // Transformar pose
            // a) obtener rotación de la transformación
            Eigen::Matrix4f rotation;
            rotation = transformation;
            rotation.col(3) = Eigen::Vector4f(0, 0, 0, 1);
            // b) Normalizar normal
            Eigen::Vector4f surface_normal;
            surface_normal << patch_plane_coefs->values[0], patch_plane_coefs->values[1], patch_plane_coefs->values[2], 1;
            surface_normal.normalize();
            // c) rotar normal
            surface_normal = rotation*surface_normal;
            // d) Obtener quaternion a partir de la normal
            pose_out.pose.orientation = Util::coefsToQuaternionMsg(surface_normal[0], surface_normal[1], surface_normal[2]);
            // e) Transportar posición
            Eigen::Vector4f surface_centroid;
            surface_centroid << cm_proj.x, cm_proj.y, cm_proj.z, 1;
            surface_centroid = transformation*surface_centroid;
            // f) guardar posición
            pose_out.pose.position.x = surface_centroid[0];
            pose_out.pose.position.y = surface_centroid[1];
            pose_out.pose.position.z = surface_centroid[2];
            pose_out.header.frame_id = Util::BASE_FRAME;
            ROS_INFO("UTIL: Pose guardada, terminando");
            return true;
        }
    }
    return false;
}

// PRIVATE
bool Util::isPointInsideBox(PointXYZ p, Box box){
    bool in_x = (p.x > box.center[0] - box.size[0]/2.0) and (p.x < box.center[0] + box.size[0]/2.0);
    bool in_y = (p.y > box.center[1] - box.size[1]/2.0) and (p.y < box.center[1] + box.size[1]/2.0);
    bool in_z = (p.z > box.center[2] - box.size[2]/2.0) and (p.z < box.center[2] + box.size[2]/2.0);
    return in_x and in_y and in_z;
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
    const double PI = 3.1416;
    Eigen::Vector3f normal = Eigen::Vector3f(coefs->values[0], coefs->values[1], coefs->values[2]);
    normal.normalize();
    // cout << "Normal: " << normal << endl;
    bool side;
    // Iterar sobre los puntos
    for (int i=0; i<cloud->points.size(); i++){
        // Calcular ángulo entre punto y normal
        Eigen::Vector3f delta = Eigen::Vector3f (p_plane.x, p_plane.y, p_plane.z) - Eigen::Vector3f(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
        delta.normalize();
        double alpha = acos(normal.dot(delta));
        // printf ("Alpha: %f°\n", (alpha*180/PI));
        if (i==0){
            side = (alpha < PI/2.0);
            // printf("Lado escogido: %s", side ? "true": "false");
            continue;
        }
        if (side != (alpha < PI/2.0)){
            // printf("Nube es cortada por plano\n");
            return true;
        }
    }
    // printf("Nube NO es cortada por plano\n");
    return false;
}