#include "Viewer.h"

Viewer::Viewer(float r, float g, float b){
	// viewer_ visualization::PCLVisualizer("3D Viewer");
	viewer_->setBackgroundColor(r, g, b);
    viewer_->addCoordinateSystem(1.0);
    viewer_->initCameraParameters();
    visualizations_.push_back(pcl::visualization::PCL_VISUALIZER_REPRESENTATION_POINTS);
    visualizations_.push_back(pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME);
    visualizations_.push_back(pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE);
    n_text_ = 0;
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

// --- Métodos para dibujar --- //
void Viewer::drawPoint(PointXYZ p, const string shape_id, float r, float g, float b, float size){
    viewer_->addSphere(p, size, r, g, b, shape_id, 0);
}
void Viewer::drawPointCloud(PointCloud<PointXYZ>::Ptr cloud, const string shape_id, float r, float g, float b, int point_size){
	viewer_->addPointCloud<PointXYZ>(cloud, shape_id);
    viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, shape_id);
    viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, shape_id);
}
void Viewer::drawPolygonMesh(PolygonMesh mesh, const string shape_id){
	viewer_->addPolygonMesh(mesh, shape_id, 0);
    viewer_->setRepresentationToWireframeForAllActors();
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
    // agregar normales al visualizador
    PointCloud<PointXYZ>::ConstPtr constcloud = centroidcloud;
    PointCloud<Normal>::ConstPtr constnormals = normals;
    viewer_->addPointCloudNormals<PointXYZ, Normal> (constcloud, constnormals, 1, 10, shape_id, 0);
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