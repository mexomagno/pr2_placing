/*
Relevante: http://www.pcl-users.org/Calculating-plane-normals-td4033575.html
*/
#include <iostream>
#include <sstream>
#include <vector>
#include <cmath>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
//#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/centroid.h>
//#include <pcl/point_types.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/filters/passthrough.h>
//#include <pcl/filters/project_inliers.h>
//#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/PolygonMesh.h>
#include <pcl/filters/project_inliers.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using namespace std;

// CONSTANTES

// VARIABLES GLOBALES
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

// METODOS

//--------- BEGIN VISUALIZADOR ---------
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (PointCloud::ConstPtr cloud){
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0.7,0.7,0.7);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    return (viewer);
}
boost::shared_ptr<pcl::visualization::PCLVisualizer> meshVis (){
    // --------------------------------------------
    // -----Open 3D viewer and add mesh-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0.7,0.7,0.7);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    return (viewer);
}
void setVisualizationType(const string nombre, int vis){
    /*
    0: points
    1: wireframe
    2: surface
    */
    vector<double> visualizations(3);
    visualizations[0] = pcl::visualization::PCL_VISUALIZER_REPRESENTATION_POINTS;
    visualizations[1] = pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME;
    visualizations[2] = pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE;
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, visualizations[vis], nombre, 0);  
}
void drawPoint(pcl::PointXYZ p, const string nombre, float r, float g, float b){
    viewer->addSphere(p, 2, r, g, b, nombre, 0);
}

void drawPolygon(pcl::PolygonMesh mesh, pcl::Vertices poly, float r, float g, float b){
    PointCloud cloudhull;
    pcl::fromPCLPointCloud2(mesh.cloud, cloudhull);
    // Crear nueva nube de puntos que represente al polígono
    //cout << "Dibujando polígono de " << poly.vertices.size() << " vértices" << endl;
    PointCloud::Ptr polygon (new PointCloud);
    polygon->width = 3;//(int)poly.vertices.size();
    polygon->height = 1;
    for (int i=0; i<(int)poly.vertices.size(); i++){
        polygon->points.push_back(cloudhull.points[poly.vertices[i]]);
    }
    // Visualizar polígono
    const string nombre="Poligono";
    PointCloud::ConstPtr const_polygon = polygon;
    viewer->addPolygon<pcl::PointXYZ>(const_polygon, r, g, b, nombre, 0);
    // Mostrar como polígono relleno
    setVisualizationType(nombre, 2);
}
//--------- END VISUALIZADOR -----------
void triangleAreaAndNormal(pcl::PointXYZ p1, pcl::PointXYZ p2, pcl::PointXYZ p3, double &area, Eigen::Vector3f &normal){
    /* Calcula el área de un triángulo definido por tres puntos */
    // Lo siguiente es una implementación de la fórmula del half-cross product: S=|ABxAC|/2
    // printf("p1: (%f,%f,%f), p2: (%f,%f,%f), p3: (%f,%f,%f)\n", p1.x, p1.y, p1.z, p2.x, p2.y, p2.z, p3.x, p3.y, p3.z);
    Eigen::Vector3f ab (p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
    Eigen::Vector3f bc (p3.x - p1.x, p3.y - p1.y, p3.z - p1.z);
    // printf("ab: (%f,%f,%f), bc: (%f,%f,%f)\n", ab[0], ab[1], ab[2], bc[0], bc[1], bc[2]);
    Eigen::Vector3f cross = ab.cross(bc);
    // printf("ab x bc: (%f,%f,%f)\n",cross[0], cross[1], cross[2]);
    area = cross.norm();
    normal = cross.normalized();
    // printf("Area: %f, normal: (%f,%f,%f)\n", area, normal[0], normal[1], normal[2]);
    //return sqrt(pow(x2*y3-x3*y2,2)+pow(x3*y1-x1*y3,2)+pow(x1*y2-x2*y1,2))/2;
}
void biggestPolygon(pcl::PolygonMesh mesh, pcl::Vertices &polygon, Eigen::Vector3f &normal, double &area){
    /* Recibe polygonmesh, recorre exhaustivamente los polígonos y retorna el más grande */
    // Contenedor de la solución. Variará a medida que avanza el algoritmo
    pcl::Vertices biggest_polygon;
    double biggest_area = 0;
    pcl::PointXYZ p1, p2, p3;
    // Contenedor de la nube de puntos del polygonmesh
    Eigen::Vector3f biggest_normal;
    PointCloud cloud;
    pcl::fromPCLPointCloud2(mesh.cloud, cloud);
    for (int pol_index = 0; pol_index < mesh.polygons.size(); pol_index++){
        p1 = cloud[mesh.polygons[pol_index].vertices[0]];
        p2 = cloud[mesh.polygons[pol_index].vertices[1]];
        p3 = cloud[mesh.polygons[pol_index].vertices[2]];
        Eigen::Vector3f current_normal;
        double current_area = 0;
        triangleAreaAndNormal(p1, p2, p3, current_area, current_normal);
        if (current_area > biggest_area){
            biggest_area = current_area;
            biggest_polygon = mesh.polygons[pol_index];
            biggest_normal = current_normal;
        }
    }
    cout << "Area de mayor polígono: " << biggest_area << endl;
    polygon = biggest_polygon;
    normal = biggest_normal;
    area = biggest_area;
}

pcl::PointXYZ getCentroid(pcl::PolygonMesh mesh){
  /*  Obtiene una aproximacion del centro de masa del polygonmesh recibido.
    Implementación actual busca centroide. */
    // Convertir puntos de malla a PointCloud
    PointCloud::Ptr meshcloud (new PointCloud);
    fromPCLPointCloud2(mesh.cloud,*meshcloud);
    Eigen::Vector4f centroid;
    compute3DCentroid(*meshcloud, centroid);
    return pcl::PointXYZ(centroid(0), centroid(1), centroid(2));
}

pcl::PointXYZ getMassCenter(pcl::PolygonMesh mesh, double biggest_area){
    /* Algoritmo:
            - Calcular centro para cada polígono
            - Calcular centroide de estos puntos, ponderado por área de los polígonos

    http://www.gamedev.net/topic/533590-how-to-find-the-center-point-of-a-convex-hull/
    */
    // obtener pointcloud del mesh y crear contenedor de centros ponderados
    PointCloud::Ptr meshcloud (new PointCloud());
    fromPCLPointCloud2(mesh.cloud,*meshcloud);
    Eigen::Vector3f suma(0,0,0);
    double areas_sum = 0;
    // calcular centro para cada polígono, ponderado por su área, y añadirlo a nube de centros ponderados
    for (int i=0; i<mesh.polygons.size(); i++){
        // Calcular centroide
        Eigen::Vector3f centroid(0,0,0);
        if (mesh.polygons[i].vertices.size() != 3)
            printf("WARNING: polígono no es un triángulo! puede suceder cualquier cosa... (%d lados)\n",(int)mesh.polygons[i].vertices.size());
        for (int j=0; j<mesh.polygons[i].vertices.size(); j++){
            pcl::PointXYZ punto = meshcloud->points[mesh.polygons[i].vertices[j]];
            centroid+=Eigen::Vector3f(punto.x, punto.y, punto.z);
        }
        centroid/=mesh.polygons[i].vertices.size();
        // Obtener area
        double area;
        Eigen::Vector3f normal; //de relleno, no se usará.
        // Asumiendo que son triángulos
        triangleAreaAndNormal(meshcloud->points[mesh.polygons[i].vertices[0]], meshcloud->points[mesh.polygons[i].vertices[1]], meshcloud->points[mesh.polygons[i].vertices[2]], area, normal);
        // Añadir para centroide ponderado (centro de masa)
        suma+=Eigen::Vector3f(centroid[0],centroid[1],centroid[2])*area;
        areas_sum+=area;
    }
    // Calcular centroide de centros ponderados
    suma/=areas_sum;
    return pcl::PointXYZ(suma[0],suma[1],suma[2]);
}

pcl::PolygonMesh getConvexHull(PointCloud::Ptr cloud){
    pcl::PolygonMesh mesh;
    pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setInputCloud(cloud);
    chull.reconstruct(mesh);
    return mesh;
}

pcl::PointXYZ projectPointOverPolygon(pcl::PointXYZ punto, Eigen::Vector3f normal, pcl::Vertices poligono, pcl::PolygonMesh mesh){
    /*
        Recibe punto a proyectar, y polígono, representado por su normal, el polígono mismo y la malla con los puntos indizados por el polígono.
        Los últimos tres parámetros se necesitan para definir completamente al plano del polígono.
    */
    // Convertir cloud del mesh a PointCloud
    PointCloud::Ptr meshcloud (new PointCloud);
    fromPCLPointCloud2(mesh.cloud,*meshcloud);

    // Calcular cuarto parámetro "d" del plano
    // 1- Tomar cualquier punto dentro del polígono
    pcl::PointXYZ inlier = meshcloud->points[poligono.vertices[0]];
    //printf("Se toma inlier (%f,%f,%f)\n",inlier.x, inlier.y, inlier.z);
    // 2- Calcular d resolviendo ax + by + cz + d = 0, con (a,b,c) = normal, (x,y,z) = inlier.
    double d = -1*(normal[0]*inlier.x + normal[1]*inlier.y + normal[2]*inlier.z);
    //printf("Normal: (%f,%f,%f)\nResultado de d: %f\n",normal[0], normal[1], normal[2], d);

    // Crear coeficientes
    pcl::ModelCoefficients::Ptr coefs (new pcl::ModelCoefficients());
    coefs->values.resize(4);
    coefs->values[0] = normal[0];
    coefs->values[1] = normal[1];
    coefs->values[2] = normal[2];
    coefs->values[3] = d;
    // Proyectar
    PointCloud::Ptr cm_cloud(new PointCloud()), cm_cloud_projected(new PointCloud());
    cm_cloud->height = cm_cloud->width = 1;
    cm_cloud->points.push_back(punto);
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cm_cloud);
    proj.setModelCoefficients(coefs);
    proj.filter(*cm_cloud_projected);
    return cm_cloud_projected->points[0];
}

bool pointInPolygon(pcl::PointXYZ punto, pcl::Vertices polygon, pcl::PolygonMesh mesh){
    /* Revisa si el punto está encerrado por el polígono o no*/
    // obtener nube de puntos desde el mesh
    PointCloud::Ptr meshcloud (new PointCloud);
    fromPCLPointCloud2(mesh.cloud,*meshcloud);

    // Crear nube de puntos con polígono
    PointCloud::Ptr polygon_cloud (new PointCloud());
    polygon_cloud->width = polygon.vertices.size();
    polygon_cloud->height = 1;
    for (int i=0; i<polygon.vertices.size(); i++){
        polygon_cloud->points.push_back(meshcloud->points[polygon.vertices[i]]);
    }

    // Verificar si está dentro o fuera del polígono
    return pcl::isPointIn2DPolygon(punto, *polygon_cloud);
}

int main(int argc, char **argv){
    if (argc < 2){
        printf("Error: Debe especificar un archivo .pcd\n");
        exit(1);
    }
    // Cargar nube de puntos
    PointCloud::Ptr cloud (new PointCloud), cloud_filt (new PointCloud);
    pcl::io::loadPCDFile(argv[1],*cloud);
    // Crear visualizador
    viewer = meshVis();

    // Crear convex hull alrededor de nube
    pcl::PolygonMesh hull = getConvexHull(cloud);
    // Visualizar convex hull como wireframe
    viewer->addPolygonMesh (hull, "hull",0);
    viewer->setRepresentationToWireframeForAllActors();
    // Obtener polígono más grande y su normal
    pcl::Vertices biggest;
    Eigen::Vector3f b_normal;
    double b_area;
    biggestPolygon(hull, biggest, b_normal, b_area);

    //printf("Normal: (%f, %f, %f)\n", b_normal[0], b_normal[1], b_normal[2]);

    // Visualizar polígono
    drawPolygon(hull, biggest, 1.0, 0.0, 0.0);
    // Obtener centro de masa
    pcl::PointXYZ ct = getCentroid(hull), cm = getMassCenter(hull, b_area);
    // Mostrar centroide
    drawPoint(ct, "Centroide", 0.0, 1.0, 0.0);
    // Mostrar centro de masa
    drawPoint(cm, "Centro de Masa", 1.0, 0.5, 0.0);
    // Proyectar ambos sobre plano del polígono
    pcl::PointXYZ ct_projected = projectPointOverPolygon(ct, b_normal, biggest, hull);
    pcl::PointXYZ cm_projected = projectPointOverPolygon(cm, b_normal, biggest, hull);
    // Mostrar ambas proyecciones
    drawPoint(ct_projected, "ct proyectado", 0.5, 1.0, 0.5);
    drawPoint(cm_projected, "cm proyectado", 1.0, 0.75, 0.5);
    // Ver posición de las proyecciones
    printf("Centroide está %s del polígono\n", (pointInPolygon(ct_projected, biggest, hull) ? "DENTRO":"FUERA"));
    printf("Centro de Masa está %s del polígono\n", (pointInPolygon(cm_projected, biggest, hull) ? "DENTRO":"FUERA"));
    while (!viewer->wasStopped()){
        viewer->spinOnce(100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}