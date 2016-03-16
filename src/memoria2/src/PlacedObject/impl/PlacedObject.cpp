#include "../PlacedObject.h"

PlacedObject::PlacedObject(){

}

void PlacedObject::setCloud(PointCloud<PointXYZ>::Ptr new_object_pc){
	object_pc = new_object_pc;


    // Construir nube que modela al gripper 
    ROS_INFO("PLACEDOBJECT: Creando nube que modela al gripper");
    // if (cloud_in->header.frame_id.find(Util::GRIPPER_FRAME_SUFFIX) == string::npos){
    //     ROS_ERROR("UTIL: No se puede filtrar gripper: Frame de nube es incorrecto");
    //     ROS_ERROR("UTIL: Frame debe contener '%s' pero era '%s'", Util::GRIPPER_FRAME_SUFFIX.c_str(), cloud_in->header.frame_id.c_str());
    //     return false;
    // }
    ROS_INFO("PLACEDOBJECT: Creando boxes");
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

    // Crear nube de puntos para cada box
    ROS_INFO("PLACEDOBJECT: Creando nubes de puntos para cada box");
    PointCloud<PointXYZ>::Ptr new_gripper_pc (new PointCloud<PointXYZ>());
    new_gripper_pc->header.frame_id = object_pc->header.frame_id;
    for (int i=0; i < gripper_boxes.size(); i++){
        // Agregar un punto en cada esquina del box
        for (int j = 0; j < 8; j++){
            PointXYZ new_point;
            new_point.x = gripper_boxes[i].center[0] + (gripper_boxes[i].size[0]/2.0)*((j/4) % 2 == 0 ? -1 : 1);
            new_point.y = gripper_boxes[i].center[1] + (gripper_boxes[i].size[1]/2.0)*((j/2) % 2 == 0 ? -1 : 1);
            new_point.z = gripper_boxes[i].center[2] + (gripper_boxes[i].size[2]/2.0)*(j     % 2 == 0 ? -1 : 1);
            new_gripper_pc->points.push_back(new_point);
        }
    }
	gripper_pc = new_gripper_pc;
}
void PlacedObject::setBaseArea(float new_base_area){
	base_area = new_base_area;
}
// void PlacedObject::setBoundingBox(vector<PointXYZ> new_bounding_box){
//     bounding_box = new_bounding_box;
// }

bool PlacedObject::computeStablePose(){
	geometry_msgs::PoseStamped pose_out;
	ROS_INFO("PLACEDOBJECT: Realizando cálculo de pose estable");
	    /*    // Obtener mejor superficie
    if (not Util::getStablePose(object_pc, gripper_pc, pose_out)){
        ROS_ERROR("PLACE: Algo ocurrió al intentar obtener la pose estable");
        return false;
    }
    return true;*/
        // Verificar que ambas nubes están en el mismo frame
    if (object_pc->header.frame_id.compare(gripper_pc->header.frame_id) != 0){
        ROS_ERROR("PLACEDOBJECT: Nube de objeto y de gripper deben estar en el mismo frame de referencia");
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
            base_area = best_patch_area;
            ROS_INFO("PLACEDOBJECT: Se ha encontrado un plano estable (de area %.2f)\n", best_patch_area);
            // Ajustar dirección de la normal de la pose (debe apuntar hacia dentro del gripper)
            float alpha = Util::angleBetweenVectors(cm_proj.x, cm_proj.y, cm_proj.z, patch_plane_coefs->values[0], patch_plane_coefs->values[1], patch_plane_coefs->values[2]);
            ROS_DEBUG("PLACEDOBJECT: pose %s será invertida", alpha < Util::PI/2.0 ? "SI" : "NO");
            int invert = (alpha < Util::PI/2.0 ? -1 : 1);
            // guardar pose
            pose_out.pose.position.x = cm_proj.x;
            pose_out.pose.position.y = cm_proj.y;
            pose_out.pose.position.z = cm_proj.z;
            pose_out.pose.orientation = Util::coefsToQuaternionMsg(patch_plane_coefs->values[0]*invert, patch_plane_coefs->values[1]*invert, patch_plane_coefs->values[2]*invert);
            pose_out.header.frame_id = object_pc->header.frame_id; // frame del gripper "tool"
            stable_pose = pose_out;
            // Guardar bounding box
            // setBoundingBox(mesh.getBoundingBox());
            computeFeatures();
            ROS_INFO("PLACEDOBJECT: Pose guardada, terminando");
            return true;
        }
    }
    return false;
}

void PlacedObject::computeFeatures(){
    MomentOfInertiaEstimation<PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(object_pc);
    feature_extractor.compute();
    PointXYZ min_obb, max_obb;
    PointXYZ position_obb;
    Eigen::Matrix3f rotation_obb;
    feature_extractor.getOBB(min_obb, max_obb, position_obb, rotation_obb);
    bounding_box.min = min_obb;
    bounding_box.max = max_obb;
    bounding_box.position.x = position_obb.x;
    bounding_box.position.y = position_obb.y;
    bounding_box.position.z = position_obb.z;
    Eigen::Quaternionf temp_q (rotation_obb);
    bounding_box.rotation.x = temp_q.x();
    bounding_box.rotation.y = temp_q.y();
    bounding_box.rotation.z = temp_q.z();
    bounding_box.rotation.w = temp_q.w();
}