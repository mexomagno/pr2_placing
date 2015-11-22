#include "Viewer.h"

Viewer::Viewer(float r, float g, float b){
	// viewer_ visualization::PCLVisualizer("3D Viewer");
    boost::shared_ptr<pcl::visualization::PCLVisualizer> vw (new visualization::PCLVisualizer ("3D Viewer"));
	viewer_ = vw;
    viewer_->setBackgroundColor(r, g, b);
    viewer_->addCoordinateSystem(1.0);
    viewer_->initCameraParameters();
    visualizations_.push_back(pcl::visualization::PCL_VISUALIZER_REPRESENTATION_POINTS);
    visualizations_.push_back(pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME);
    visualizations_.push_back(pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE);
    n_text_ = 0;
    FONT_SIZE = 15;
}

// Setea puntos - wireframe - surfaces
void Viewer::setVisualizationMode(const string shape_id, int mode){
	/*
    0: points
    1: wireframe
    2: surface
    */
   	viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, visualizations_[mode], shape_id, 0);  
}
void Viewer::setLineWidth(const string shape_id, int width){
    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, width, shape_id, 0);  
}
void Viewer::setFontSize(const string shape_id, int size){
    FONT_SIZE = size;
}
void Viewer::setColor(const string shape_id, float r, float g, float b, bool iscloud){
    if (iscloud)
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, shape_id, 0);  
    else
        viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, shape_id, 0);  
}
// --- Métodos para dibujar --- //
void Viewer::drawPoint(PointXYZ p, PointCloud<PointXYZ>::Ptr cloud, const string shape_id, float r, float g, float b){
    // Calcular tamaño de normales, basado en tamaño total de la nube
    MomentOfInertiaEstimation<PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cloud);
    feature_extractor.compute();
    PointXYZ min_obb, max_obb;
    PointXYZ position_obb;        // no se usará
    Eigen::Matrix3f rotation_obb; // no se usará
    feature_extractor.getOBB(min_obb, max_obb, position_obb, rotation_obb);
    double dx = abs(min_obb.x - max_obb.x);
    double dy = abs(min_obb.y - max_obb.y);
    double dz = abs(min_obb.z - max_obb.z);
    double size = (dx+dy+dz)/3.0*POINT_SIZE_PONDERATOR;
    viewer_->addSphere(p, size, r, g, b, shape_id, 0);
}
void Viewer::drawPoint(PointXYZ p, const string shape_id, float r, float g, float b){
    // Calcular tamaño de normales, basado en tamaño total de la nube
    viewer_->addSphere(p, POINT_SIZE_PONDERATOR*20, r, g, b, shape_id, 0);
}
void Viewer::drawPointCloud(PointCloud<PointXYZ>::Ptr cloud, const string shape_id, float r, float g, float b, int point_size){
	viewer_->addPointCloud<PointXYZ>(cloud, shape_id);
    viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, shape_id);
    viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, shape_id);
}

void Viewer::drawPolygonMesh(PolygonMesh mesh, const string shape_id, float r, float g, float b, float a){
	viewer_->addPolygonMesh(mesh, shape_id, 0);
    //viewer_->setRepresentationToWireframeForAllActors();
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, a, shape_id);
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, shape_id);
}
void Viewer::drawPolygon(Vertices polygon, PolygonMesh mesh, const string shape_id, float r, float g, float b, bool filled){
	PointCloud<PointXYZ> cloudhull;
    pcl::fromPCLPointCloud2(mesh.cloud, cloudhull);
    // Crear nueva nube de puntos que represente al polígono
    PointCloud<PointXYZ>::Ptr polycloud (new PointCloud<PointXYZ>());
    polycloud->width  = (int)polygon.vertices.size();
    polycloud->height = 1;
    for (int i=0; i<(int)polygon.vertices.size(); i++){
        polycloud->points.push_back(cloudhull.points[polygon.vertices[i]]);
    }
    // Visualizar polígono
    PointCloud<PointXYZ>::ConstPtr const_polygon = polycloud;
    viewer_->addPolygon<pcl::PointXYZ>(const_polygon, r, g, b, shape_id, 0);
    // Mostrar como polígono relleno
    setVisualizationMode(shape_id, (filled ? 2 : 1));
}
void Viewer::drawPolygonVector(vector<int> polygon, PolygonMesh mesh, const string shape_id_prefix, float in_r, float in_g, float in_b, float out_r, float out_g, float out_b, int width, float alpha){
    for (int i=0; i<polygon.size(); i++){
        stringstream patch_id, patch_wire_id;
        patch_id << shape_id_prefix << "_" << i;
        patch_wire_id << shape_id_prefix << "_wire_" << i;
        this->drawPolygon(mesh.polygons[polygon[i]], mesh, patch_id.str(), in_r, in_g, in_b);
        this->drawPolygon(mesh.polygons[polygon[i]], mesh, patch_wire_id.str(), out_r, out_g, out_b, false);
        this->setLineWidth(patch_wire_id.str(), width);
        viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, alpha, patch_id.str());
        viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, alpha, patch_wire_id.str());
    }
}

void Viewer::drawPolygonMeshNormals(Polymesh mesh, const string shape_id, float r, float g, float b){
	// Obtener nube de puntos
    PointCloud<PointXYZ>::Ptr centroidcloud (new PointCloud<PointXYZ>());
    centroidcloud->height = 1;
    centroidcloud->width = mesh.getPolygonNumber();
    // Crear contenedor de normales
    PointCloud<Normal>::Ptr normals (new PointCloud<Normal>());
    normals->height = 1;
    normals->width = mesh.getPolygonNumber();
    // Rellenar contenedor con normales
    for (int i=0; i< mesh.getPolygonNumber(); i++){
        Eigen::Vector3f normal = mesh.getNormal(i);
        PointXYZ centroid = mesh.getCentroid(i);
        // añadir normal a normales
        normals->points.push_back(Normal(normal[0], normal[1], normal[2]));
        // Añadir centroide
        centroidcloud->points.push_back(PointXYZ(centroid.x, centroid.y, centroid.z));
    }
    // Calcular tamaño de normales, basado en tamaño total de la nube
    MomentOfInertiaEstimation<PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(mesh.getPointCloud());
    feature_extractor.compute();
    PointXYZ min_obb, max_obb;
    PointXYZ position_obb;        // no se usará
    Eigen::Matrix3f rotation_obb; // no se usará
    feature_extractor.getOBB(min_obb, max_obb, position_obb, rotation_obb);
    double dx = abs(min_obb.x - max_obb.x);
    double dy = abs(min_obb.y - max_obb.y);
    double dz = abs(min_obb.z - max_obb.z);
    double normal_size = (dx+dy+dz)/3.0*NORMAL_SIZE_PONDERATOR;
    // agregar normales al visualizador
    PointCloud<PointXYZ>::ConstPtr constcloud = centroidcloud;
    PointCloud<Normal>::ConstPtr constnormals = normals;
    viewer_->addPointCloudNormals<PointXYZ, Normal> (constcloud, constnormals, 1, normal_size, shape_id, 0);
    viewer_->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_COLOR, r, g, b, shape_id, 0);
/*  
    // Dibujar puntos en centroides de los polígonos
    for (int i=0; i<centroidcloud->points.size(); i++){
        stringstream pointname;
        pointname << "centroid_nr_" << i;
        drawPoint(centroidcloud->points[i], pointname.str(), NORMALS_COLOR[0], NORMALS_COLOR[1], NORMALS_COLOR[2]);
    }*/ 
}
void Viewer::addText(const string text, const string shape_id, float r, float g, float b){
    viewer_->addText(text, LEFT_MARGIN, BOTTOM_MARGIN+(FONT_SIZE+1)*n_text_, FONT_SIZE, r, g, b, shape_id, 0);
    n_text_++;
}
void Viewer::show(){    
    while (!viewer_->wasStopped()){
        viewer_->spinOnce(100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}