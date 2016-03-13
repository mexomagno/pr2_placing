#include "../PlacedObject.h"

PlacedObject::PlacedObject(){

}

void PlacedObject::setClouds(PointCloud<PointXYZ>::Ptr new_object_pc, PointCloud<PointXYZ>::Ptr new_gripper_pc){
	object_pc = new_object_pc;
	gripper_pc = new_gripper_pc;
}
void PlacedObject::setBaseArea(float new_base_area){
	base_area = new_base_area;
}
bool PlacedObject::computeStablePose(){
	geometry_msgs::PoseStamped pose_out;
	ROS_INFO("PLACEDOBJECT: Acá se debiera computar la pose estable");
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
            ROS_INFO("PLACEDOBJECT: Pose guardada, terminando");
            return true;
        }
    }
    return false;

}