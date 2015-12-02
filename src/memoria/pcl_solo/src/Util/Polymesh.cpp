#include "Polymesh.h"
// Structs para ordenar
struct PatchAndArea{
    vector<int> patch;
    double area;
    PatchAndArea(vector<int> p, double a) : patch(p), area(a) {}
    bool operator <( const PatchAndArea &val) const{
        return area < val.area;
    }
};


// PUBLIC
Polymesh::Polymesh(PolygonMesh mesh){
	mesh_ = mesh;
	poly_number_ = mesh.polygons.size();
	// Obtener Pointcloud
	PointCloud<PointXYZ>::Ptr cld (new PointCloud<PointXYZ>());
	meshcloud_ = cld;
	fromPCLPointCloud2(mesh.cloud, *meshcloud_);
    // Obtener centroide de la nube
    ct_ = this->computeCentroid();
	// Obtener normales, areas y centroides
	for (int i=0; i<poly_number_; i++){
		double poly_area;
		Eigen::Vector3f poly_normal;
		PointXYZ poly_centroid, p1, p2, p3, ptest;
		p1 = meshcloud_->points[mesh_.polygons[i].vertices[0]];
		p2 = meshcloud_->points[mesh_.polygons[i].vertices[1]];
		p3 = meshcloud_->points[mesh_.polygons[i].vertices[2]];
		triangleAreaNormalCentroid(p1, p2, p3, poly_area, poly_normal, poly_centroid);
		// Añadir al listado
		poly_areas_.push_back(poly_area);
		poly_normals_.push_back(poly_normal);
		poly_centroids_.push_back(poly_centroid);
	}
    // Obtener centro de masa
    cm_ = getCenterOfMassInternal();
}
PointXYZ Polymesh::computeCentroid(){
	Eigen::Vector3f accum (0,0,0);
	for (int i=0; i<meshcloud_->points.size(); i++){
		PointXYZ meshpoint = meshcloud_->points[i];
		accum += Eigen::Vector3f(meshpoint.x, meshpoint.y, meshpoint.z);
	}
	accum /= meshcloud_->points.size();
	return PointXYZ(accum[0], accum[1], accum[2]);
}
PointXYZ Polymesh::getMeshCentroid(){
    return ct_;
}
void Polymesh::getBiggestPolygon(Vertices &polygon, int &poly_index, double &area, Eigen::Vector3f &normal, PointXYZ &centroid){
    /* Recibe polygonmesh, recorre exhaustivamente los polígonos y retorna el más grande.
    */
    // Contenedor de la solución. Variará a medida que avanza el algoritmo
    double biggest_area = 0;
    double biggest_index = 0;
    for (int i=0; i<poly_number_; i++){
        if (poly_areas_[i] > biggest_area){
            biggest_area = poly_areas_[i];
            biggest_index = i;
        }
    }
    polygon    = mesh_.polygons[biggest_index];
    area       = poly_areas_[biggest_index];
    normal     = poly_normals_[biggest_index];
    centroid   = poly_centroids_[biggest_index];
    poly_index = biggest_index;
}
void Polymesh::getBiggestFlatPatch(double angle_threshold, vector<int> &patch){
    /* 
        Esta función tiene como fin buscar el mayor parche plano del convex hull.
        Es importante tener en cuenta que en un objeto no convexo, podría darse la situación
        en que dos grupos distintos de polígonos podrían tener normales similares pero ser disconexos.
        En el caso del convex hull esto NO es posible.
        Para esto:
            - Recorre polígonos de convex hull
            - Los separa por diferencia de normales (si tienen normales muy distintas, son de grupos distintos)
            - Escoge grupo con mayor área

        Detalle:
            Hull tiene N polígonos
            - Crear matriz vacía tamaño N x ? para almacenar grupos (? es variable)
            - crear vector en 0 tamaño N para almacenar area total para cada grupo
            - Para cada polígono:
                - Iniciar nuevo grupo vacío tamaño variable.
                - Iniciar contador de area total
                - Para cada otro polígono:
                    - Si tiene normal similar:
                        - añadir su índice al grupo
                        - Sumar su área al area total
                - Añadir grupo a lista de grupos
                - Añadir area a lista de areas de grupos
            - Buscar area mas grande, retornar grupo asociado

        Los grupos: 
            Los grupos son unidades que almacenan N polígonos y representan un "parche" plano. Además tienen asociada un área total, almacenada en otro arreglo.
            La estructura de cada grupo es un vector de tamaño variable con los polígonos, representados por su índice dentro del mesh.polygons.
            Las areas totales están en otro vector, de tamaño igual a la cantidad de grupos.

            IMPORTANTE: Existe una correspondencia 1:1 entre los índices de: "patches", "areas" y mesh.polygons

        TODO:
            - Reparar problemas por orientación de normales (Polígonos caras opuestas, polígonos ignorados)
            - Revisar que centro de masa se proyecte dentro del grupo
            - Revisar que el grupo sea factible de posicionar según agarre del gripper
            - Aplicar optimizaciones propuestas
    */
    vector<vector<int> > patches;
    vector<double> areas;
    this->getFlatPatches(angle_threshold, patches, areas);
    // En este punto, "patches" y "areas" son de tamaño "poly_number_"
    // Buscar area más grande
    double biggest_area = 0;
    int biggest_index = 0;
    for (int i=0; i<poly_number_; i++){
        if (areas[i] > biggest_area){
            biggest_area = areas[i];
            biggest_index = i;
        } 
    }
    // Retornar parche más grande
    for (int i=0; i<patches[biggest_index].size(); i++){
        //patch.push_back(mesh_.polygons[patches[biggest_index][i]]);
        patch.push_back(patches[biggest_index][i]);
    }
}
void Polymesh::getFlatPatches(double angle_threshold, vector<vector<int> > &patches, vector<double> &areas){
    // Recorrer polígonos y crear patch para cada uno
    for (int i=0; i<poly_number_; i++){
        // Crear su grupo y añadirse a si mismo
        vector<int> patch;
        patch.push_back(i);
        double patch_area = poly_areas_[i];
        // Añadir todos los parches de normal similar
        for (int j=0; j<poly_number_; j++){
            // Descartarse a si mismo
            if (i==j)
                continue;
            // Comparar diferencia de normal. Notar que ya vienen normalizadas
            double anglediff = acos(poly_normals_[i].dot(poly_normals_[j]));
            if (anglediff < angle_threshold){
                // Agregar al parche
                patch.push_back(j);
                patch_area += poly_areas_[j];
            }
        }
        // Añadir grupo y area a arreglos externos
        patches.push_back(patch);
        areas.push_back(patch_area);
    }
    // En este punto, "patches" y "areas" son de tamaño "poly_number_"

    // Ordenar de mayor a menor area
    vector<PatchAndArea> v;
    for (int i=0; i<patches.size(); i++){
        v.push_back(PatchAndArea(patches[i], areas[i]));
    }
    sort(v.begin(), v.end());
    // Reconsruir arreglos
    vector<vector<int> > patches_s;
    vector<double> areas_s;
    for (int i=v.size()-1; i>=0; i--){
        patches_s.push_back(v[i].patch);
        areas_s.push_back(v[i].area);
    }
    patches = patches_s;
    areas = areas_s;
}
PolygonMesh Polymesh::getPCLMesh(){
    return mesh_;
}
PointCloud<PointXYZ>::Ptr Polymesh::getPointCloud(){
    return meshcloud_;
}

int Polymesh::getPolygonNumber(){
    return poly_number_;
}
double Polymesh::getArea(int index){
    return poly_areas_[index];
}
Eigen::Vector3f Polymesh::getNormal(int index){
    return poly_normals_[index];
}
PointXYZ Polymesh::getCentroid(int index){
    return poly_centroids_[index];
}
PointXYZ Polymesh::getCenterOfMass(){
    return cm_;
}

// Utilidades
PointXYZ Polymesh::projectPointOverPolygon(PointXYZ p, int poly_index){
    /*
        Recibe punto a proyectar, y polígono, representado por su normal, el polígono mismo y la malla con los puntos indizados por el polígono.
        Los últimos tres parámetros se necesitan para definir completamente al plano del polígono.
    */
    // Calcular cuarto parámetro "d" del plano
    // 1- Tomar cualquier punto dentro del polígono
    PointXYZ inlier = meshcloud_->points[mesh_.polygons[poly_index].vertices[0]];
    // 2- Calcular d resolviendo ax + by + cz + d = 0, con (a,b,c) = normal, (x,y,z) = inlier.
    double d = -1*(poly_normals_[poly_index][0]*inlier.x + poly_normals_[poly_index][1]*inlier.y + poly_normals_[poly_index][2]*inlier.z);
    // Crear coeficientes
    ModelCoefficients::Ptr coefs (new ModelCoefficients());
    coefs->values.resize(4);
    coefs->values[0] = poly_normals_[poly_index][0];
    coefs->values[1] = poly_normals_[poly_index][1];
    coefs->values[2] = poly_normals_[poly_index][2];
    coefs->values[3] = d;
    // Proyectar
    PointCloud<PointXYZ>::Ptr cm_cloud(new PointCloud<PointXYZ>()), cm_cloud_projected(new PointCloud<PointXYZ>());
    cm_cloud->height = cm_cloud->width = 1;
    cm_cloud->points.push_back(p);
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cm_cloud);
    proj.setModelCoefficients(coefs);
    proj.filter(*cm_cloud_projected);
    return cm_cloud_projected->points[0];
}
PointXYZ Polymesh::projectPointOverFlatPointCloud(PointXYZ p, PointCloud<PointXYZ>::Ptr cloud){
    // 1- Tomar cualquier punto dentro del polígono
    PointXYZ inlier = cloud->points[0];
    // 2- Obtener coeficientes del plano representado por el cloud. Para eso, tomar un triángulo del cloud.
    int index_0 = floor(cloud->points.size()/3)*1 - 1;
    int index_1 = floor(cloud->points.size()/3)*2 - 1;
    int index_2 = floor(cloud->points.size()/3)*3 - 1;
    Eigen::Vector3f normal = triangleNormal(cloud->points[index_0], cloud->points[index_1], cloud->points[index_2]);
    double d = -1*(normal[0]*inlier.x + normal[1]*inlier.y + normal[2]*inlier.z);
    ModelCoefficients::Ptr coefs (new ModelCoefficients());
    coefs->values.resize(4);
    coefs->values[0] = normal[0];
    coefs->values[1] = normal[1];
    coefs->values[2] = normal[2];
    coefs->values[3] = d;
    // 3- Proyectar
    PointCloud<PointXYZ>::Ptr p_cloud(new PointCloud<PointXYZ>()), p_cloud_projected(new PointCloud<PointXYZ>());
    p_cloud->height = p_cloud->width = 1;
    p_cloud->points.push_back(p);
    ProjectInliers<PointXYZ> proj;
    proj.setModelType(SACMODEL_PLANE);
    proj.setInputCloud(p_cloud);
    proj.setModelCoefficients(coefs);
    proj.filter(*p_cloud_projected);
    return p_cloud_projected->points[0];    
}
bool Polymesh::pointInPolygon(PointXYZ p, int poly_index){
    /* Revisa si el punto está encerrado por el polígono o no*/
    // Crear nube de puntos con polígono
    Vertices polygon = mesh_.polygons[poly_index];
    int n_vertices = polygon.vertices.size();
    PointCloud<PointXYZ>::Ptr polygon_cloud (new PointCloud<PointXYZ>());
    polygon_cloud->width = n_vertices;
    polygon_cloud->height = 1;
    for (int i=0; i<n_vertices; i++){
        polygon_cloud->points.push_back(meshcloud_->points[polygon.vertices[i]]);
    }
    // Verificar si está dentro o fuera del polígono
    return isPointIn2DPolygon(p, *polygon_cloud);
}
void Polymesh::flattenPatch(vector<int> patch, PointCloud<PointXYZ> &flatcloud, ModelCoefficients::Ptr &patchcoefs){
    /*
    Esta función se encarga de transformar un parche de polígonos en un polígono bidimensional.
    En definitiva, se obtiene el plano representado por el parche de polígonos.

    Algoritmo:
        - Obtener coeficientes del plano P representado por el parche p
        - Proyectar todos los puntos del parche p sobre P y obtener p'
        - Obtener convex hull 2D de p' y obtener nube de puntos q
    */
    // OBTENER PLANO REPRESENTADO POR PARCHE
    double SEG_THRESHOLD = 0.01;
    // 1) Crear nube de puntos que represente a los parches.
    //     Esta nube está hecha de los puntos de cada polígono
    PointCloud<PointXYZ>::Ptr patch_cloud (new PointCloud<PointXYZ>());
    vector<int> already_added_index;
    for (int i=0; i<patch.size(); i++){
        // Generar lista de indices de puntos en el parche, sin repetir
        for (int j=0; j<mesh_.polygons[patch[i]].vertices.size(); j++){
            if (already_added_index.empty())
                already_added_index.push_back(mesh_.polygons[patch[i]].vertices[j]);
            if (already_added_index.end() == find(already_added_index.begin(), already_added_index.end(), mesh_.polygons[patch[i]].vertices[j]))
                already_added_index.push_back(mesh_.polygons[patch[i]].vertices[j]);
        }
    }
    // Agregar puntos a nube
    for (int i=0; i<already_added_index.size(); i++)
        patch_cloud->points.push_back(meshcloud_->points[already_added_index[i]]);
    // 2) Ajustar un plano a esta nube de puntos
    // 2.1) Obtener dimensiones del mesh
    MomentOfInertiaEstimation<PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(patch_cloud);
    feature_extractor.compute();
    PointXYZ min_obb, max_obb;
    PointXYZ position_obb;        // no se usará
    Eigen::Matrix3f rotation_obb; // no se usará
    feature_extractor.getOBB(min_obb, max_obb, position_obb, rotation_obb);
    double dx = abs(min_obb.x - max_obb.x);
    double dy = abs(min_obb.y - max_obb.y);
    double dz = abs(min_obb.z - max_obb.z);
    double dmin = min(dx, min(dy, dz));
    SEG_THRESHOLD = (dmin > SEG_THRESHOLD ? dmin : SEG_THRESHOLD);
    //printf("dx:%f, dy:%f, dz:%f\n", dx, dy, dz);
    //printf("SEG_THRESHOLD fijado en %f\n", SEG_THRESHOLD);
    // 2.2 segmentar
    SACSegmentation<PointXYZ> segmentator;
    //segmentator.setOptimizeCoefficients(true);
    segmentator.setModelType(SACMODEL_PLANE);
    segmentator.setMethodType(SAC_RANSAC);
    segmentator.setDistanceThreshold(SEG_THRESHOLD);
    segmentator.setInputCloud(patch_cloud);
    ModelCoefficients::Ptr coefs (new ModelCoefficients());
    PointIndices::Ptr inliers (new PointIndices()); // No se utilizarán
    segmentator.segment(*inliers, *coefs);
    patchcoefs = coefs;
    // OJO: El segmentador se queja cuando encuentra menos de 4 inliers, sin embargo encontrar 3 es suficiente para nosotros.
    // Tener esto en cuenta. Puede repararse agregando uno de los 3 puntos 2 veces.
    // Por otro lado, esto no quita que funcione. El error lo lanza un llamado interno a optimizeModelCoefficients().
    // Ver http://docs.pointclouds.org/1.7.0/sac__model__plane_8hpp_source.html, línea 224.
    if (inliers->indices.size() == 3){
        printf("(OJO:: El warning anterior no afecta en nada)\n");
    }
    // PROYECTAR PARCHE SOBRE EL PLANO
    PointCloud<PointXYZ>::Ptr patch_cloud_projected (new PointCloud<PointXYZ>());
    ProjectInliers<PointXYZ> projector;
    projector.setModelType(SACMODEL_PLANE);
    projector.setInputCloud(patch_cloud);
    projector.setModelCoefficients(coefs);
    projector.filter(*patch_cloud_projected);

    // OBTENER CONVEX HULL 2D DE PROYECCIÓN
    ConvexHull<PointXYZ> c_huller;
    c_huller.setInputCloud(patch_cloud_projected);
    c_huller.reconstruct(flatcloud);
}
bool Polymesh::isPointInConvexPolygon(PointXYZ p, PointCloud<PointXYZ> poly){
    return isPointIn2DPolygon(p, poly);
}



// PRIVATE
bool Polymesh::triangleAreaNormalCentroid(PointXYZ p1, PointXYZ p2, PointXYZ p3, double &area, Eigen::Vector3f &normal, PointXYZ &centroid){
    // Lo siguiente es una implementación de la fórmula del half-cross product: S=|ABxAC|/2
    Eigen::Vector3f ab (p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
    Eigen::Vector3f bc (p3.x - p1.x, p3.y - p1.y, p3.z - p1.z);
    Eigen::Vector3f cross = ab.cross(bc);
    // Almacenadores temporales
    double area_tmp = cross.norm();
    Eigen::Vector3f normal_tmp = cross.normalized();
    Eigen::Vector3f eigen_centroid = (Eigen::Vector3f(p1.x, p1.y, p1.z) + Eigen::Vector3f(p2.x, p2.y, p2.z) + Eigen::Vector3f(p3.x, p3.y, p3.z))/3;
    PointXYZ centroid_tmp = PointXYZ(eigen_centroid[0], eigen_centroid[1], eigen_centroid[2]);

    // Chequear orientación de la normal usando el punto Test
    Eigen::Vector3f delta = Eigen::Vector3f (p1.x, p1.y, p1.z) - Eigen::Vector3f (ct_.x, ct_.y, ct_.z);
    delta.normalize();
    double angle = acos(normal_tmp.dot(delta));
    if (PI/2.0-ANGLE_RELAXATION < angle and angle < PI/2.0+ANGLE_RELAXATION){
        printf("WARNING: Angulo muy cercano a 90° (%f°) puede llevar a error!\n", toGrad(angle));
    }
    if (angle > PI/2){
        normal_tmp*= -1;
    }
    area     = area_tmp;
    normal   = normal_tmp;
    centroid = centroid_tmp;
    return true;
}
Eigen::Vector3f Polymesh::triangleNormal(PointXYZ p1, PointXYZ p2, PointXYZ p3){
    /* Igual que el método anterior pero sin corrección de orientación. 
        Diseñada para obtención de coeficientes de un plano definido por tres puntos. */
    Eigen::Vector3f ab (p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
    Eigen::Vector3f bc (p3.x - p1.x, p3.y - p1.y, p3.z - p1.z);
    Eigen::Vector3f cross = ab.cross(bc);
    return cross.normalized();
}
int Polymesh::getAnyOtherIndex(int not_this){
    int randint = not_this;
    while (randint == not_this)
        randint = (int)rand() % poly_number_;
    return randint;
}
PointXYZ Polymesh::getCenterOfMassInternal(){
	/* Algoritmo:
            - Calcular centro para cada polígono
            - Calcular centroide de estos puntos, ponderado por área de los polígonos

    http://www.gamedev.net/topic/533590-how-to-find-the-center-point-of-a-convex-hull/
    */
    // obtener pointcloud del mesh y crear contenedor de centros ponderados
    Eigen::Vector3f suma(0,0,0);
    double areas_sum = 0;
    // calcular centro para cada polígono, ponderado por su área, y añadirlo a nube de centros ponderados
    for (int i=0; i<poly_number_; i++){
    	int n_vertices = mesh_.polygons[i].vertices.size();
        if (n_vertices != 3)
            printf("WARNING: polígono no es un triángulo! puede suceder cualquier cosa... (%d lados)\n",(int)mesh_.polygons[i].vertices.size());
        // Añadir para centroide ponderado (centro de masa)
        suma += Eigen::Vector3f(poly_centroids_[i].x,poly_centroids_[i].y,poly_centroids_[i].z)*poly_areas_[i];
        areas_sum += poly_areas_[i];
    }
    // Calcular centroide de centros ponderados
    suma /= areas_sum;
    return PointXYZ(suma[0],suma[1],suma[2]);
}
double Polymesh::toGrad(double rads){
    return 180.0*rads/PI;
}