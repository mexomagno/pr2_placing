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
#include <pcl/segmentation/extract_polygonal_prism_data.h> // para isPointIn2DPolygon()
#include <pcl/surface/convex_hull.h>
#include <pcl/PolygonMesh.h>
#include <pcl/filters/project_inliers.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using namespace std;

// CONSTANTES
/* --- Formatos de visualización --- */
int FONT_SIZE                   = 15;
float BG_COLOR[]                = {0.0, 0.0, 0.0};
float CM_COLOR[]                = {1.0, 0.5, 0.0};
float CT_COLOR[]                = {1.0, 0.0, 0.0};
float FLAT_SURFACE_COLOR[]      = {0.0, 1.0, 0.0};
float FLAT_SURFACE_WIRE_COLOR[] = {0.0, 0.0, 1.0};
float FLAT_SURFACE_WIRE_WIDTH   = 3;
float LIGHT_FACTOR              = 0.5;
float DARK_FACTOR               = 0.5;
int BOTTOM_MARGIN               = 15;
int LEFT_MARGIN                 = 10;
int NORMALS_COLOR[]             = {1.0, 0.0, 1.0};
/* --- Constantes para el algoritmo --- */
double PATCH_ANGLE_THRESHOLD = 0.2;
double PI = 3.1415;
// VARIABLES GLOBALES
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

// METODOS

//--------- BEGIN VISUALIZADOR ---------
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (PointCloud::ConstPtr cloud){
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (BG_COLOR[0], BG_COLOR[1], BG_COLOR[2]);
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
    viewer->setBackgroundColor (BG_COLOR[0], BG_COLOR[1], BG_COLOR[2]);
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

void drawPolygon(pcl::PolygonMesh mesh, pcl::Vertices poly, float r, float g, float b, const string name, bool filled = true){
    PointCloud cloudhull;
    pcl::fromPCLPointCloud2(mesh.cloud, cloudhull);
    // Crear nueva nube de puntos que represente al polígono
    //cout << "Dibujando polígono de " << poly.vertices.size() << " vértices" << endl;
    PointCloud::Ptr polygon (new PointCloud);
    polygon->width  = 3;//(int)poly.vertices.size();
    polygon->height = 1;
    for (int i=0; i<(int)poly.vertices.size(); i++){
        polygon->points.push_back(cloudhull.points[poly.vertices[i]]);
    }
    // Visualizar polígono
    PointCloud::ConstPtr const_polygon = polygon;
    viewer->addPolygon<pcl::PointXYZ>(const_polygon, r, g, b, name, 0);
    // Mostrar como polígono relleno
    setVisualizationType(name, (filled ? 2 : 1));
}
void triangleAreaAndNormal(pcl::PointXYZ p1, pcl::PointXYZ p2, pcl::PointXYZ p3, pcl::PointXYZ test, double &area, Eigen::Vector3f &normal);
void drawPolygonMeshNormals(pcl::PolygonMesh mesh){
    // Obtener nube de puntos
    PointCloud::Ptr meshcloud (new PointCloud()), centroidcloud (new PointCloud());
    fromPCLPointCloud2(mesh.cloud, *meshcloud);
    centroidcloud->height = 1;
    centroidcloud->width = mesh.polygons.size();
    // Crear contenedor de normales
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>());
    normals->height = 1;
    normals->width = mesh.polygons.size();
    pcl::PointXYZ p1, p2, p3, ptest;
    // Rellenar contenedor con normales
    for (int i=0; i< mesh.polygons.size(); i++){
        double area;
        p1 = meshcloud->points[mesh.polygons[i].vertices[0]];
        p2 = meshcloud->points[mesh.polygons[i].vertices[1]];
        p3 = meshcloud->points[mesh.polygons[i].vertices[2]];
        Eigen::Vector3f eigen_normal, centroid, p1_eigen(p1.x,p1.y,p1.z), p2_eigen(p2.x,p2.y,p2.z), p3_eigen(p3.x,p3.y,p3.z);
        int randint = i;
        while (randint == i)
            randint = (int)rand() % mesh.polygons.size();
        ptest = meshcloud->points[mesh.polygons[randint].vertices[0]];
        triangleAreaAndNormal(p1, p2, p3, ptest, area, eigen_normal);
        // Añadir centroide
        centroid = (p1_eigen + p2_eigen + p3_eigen)/3;
        centroidcloud->points.push_back(pcl::PointXYZ(centroid[0], centroid[1], centroid[2]));
        // añadir normal a normales
        normals->points.push_back(pcl::Normal(eigen_normal[0], eigen_normal[1], eigen_normal[2]));
    }
    // agregar normales al visualizador
    PointCloud::ConstPtr constcloud = centroidcloud;
    pcl::PointCloud<pcl::Normal>::ConstPtr constnormals = normals;
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (constcloud, constnormals, 1, 10, "normals", 0);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, NORMALS_COLOR[0], NORMALS_COLOR[1], NORMALS_COLOR[2], "normals", 0);
    for (int i=0; i<centroidcloud->points.size(); i++){
        stringstream pointname;
        pointname << "centroid_nr_" << i;
        drawPoint(centroidcloud->points[i], pointname.str(), NORMALS_COLOR[0], NORMALS_COLOR[1], NORMALS_COLOR[2]);
    } 
}
//--------- END VISUALIZADOR -----------
void triangleAreaAndNormal(pcl::PointXYZ p1, pcl::PointXYZ p2, pcl::PointXYZ p3, pcl::PointXYZ test, double &area, Eigen::Vector3f &normal){
    /* Calcula el área de un triángulo definido por tres puntos
        
    TODO:
        - Cuidar orientación de las normales
    */
    // Lo siguiente es una implementación de la fórmula del half-cross product: S=|ABxAC|/2
    Eigen::Vector3f ab (p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
    Eigen::Vector3f bc (p3.x - p1.x, p3.y - p1.y, p3.z - p1.z);
    Eigen::Vector3f cross = ab.cross(bc);
    area   = cross.norm();
    normal = cross.normalized();
    // Chequear orientación de la normal usando el punto Test
    Eigen::Vector3f delta = Eigen::Vector3f (p1.x, p1.y, p1.z) - Eigen::Vector3f (test.x, test.y, test.z);
    if (acos(normal.dot(delta)) > PI/2){
        cout << "Invirtiendo normal. Antes:" << normal << endl;
        normal*= -1;
        cout << "Ahora: " << normal << endl;
    }
}

void biggestPolygon(pcl::PolygonMesh mesh, pcl::Vertices &polygon, Eigen::Vector3f &normal, double &area){
    /* Recibe polygonmesh, recorre exhaustivamente los polígonos y retorna el más grande.

    TODO:
        - Cuidar orientación de normales, para eso, cuidar el orden con que se pasan los puntos.
    */
    // Contenedor de la solución. Variará a medida que avanza el algoritmo
    pcl::Vertices biggest_polygon;
    double biggest_area = 0;
    pcl::PointXYZ p1, p2, p3, ptest;
    // Contenedor de la nube de puntos del polygonmesh
    Eigen::Vector3f biggest_normal;
    PointCloud::Ptr cloud (new PointCloud());
    pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
    for (int pol_index = 0; pol_index < mesh.polygons.size(); pol_index++){
        p1 = cloud->points[mesh.polygons[pol_index].vertices[0]];
        p2 = cloud->points[mesh.polygons[pol_index].vertices[1]];
        p3 = cloud->points[mesh.polygons[pol_index].vertices[2]];
        // obtener índice de un polígono cualquiera
        int randint = pol_index;
        while (randint == pol_index)
            randint = (int)rand() % mesh.polygons.size();
        ptest = cloud->points[mesh.polygons[randint].vertices[0]]; // primer punto de un polígono escogido al azar
        Eigen::Vector3f current_normal;
        double current_area = 0;
        triangleAreaAndNormal(p1, p2, p3, ptest, current_area, current_normal);
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

pcl::PointXYZ getCenterOfMass(pcl::PolygonMesh mesh, double biggest_area){
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
        // obtener índice de un polígono cualquiera
        int randint = i;
        while (randint == i)
            randint = (int)rand() % mesh.polygons.size();
        triangleAreaAndNormal(meshcloud->points[mesh.polygons[i].vertices[0]], meshcloud->points[mesh.polygons[i].vertices[1]], meshcloud->points[mesh.polygons[i].vertices[2]], meshcloud->points[mesh.polygons[randint].vertices[0]], area, normal);
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

void biggestFlatPatch(pcl::PolygonMesh mesh){
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
    // Constantes
    int n_polygons = mesh.polygons.size(); // Número de polígonos en el mesh

    // Obtener pointcloud del mesh
    PointCloud::Ptr meshcloud (new PointCloud);
    fromPCLPointCloud2(mesh.cloud,*meshcloud);

    // Crear contenedor de grupos y contenedor de areas totales
    vector< vector<int> > patches; // Almacena "parches". Cada "parche" es un arreglo de índices de polígonos
    vector<double> areas;      // Almacena area acumulada para cada "parche"

    // Procesar polígonos: Obtener sus normales y areas primero que todo
    vector<Eigen::Vector3f> poly_normals;
    vector<double> poly_areas;
    for (int i=0; i<n_polygons; i++){
        Eigen::Vector3f normal;
        double area;
        // obtener índice de un polígono cualquiera
        int randint = i;
        while (randint == i)
            randint = (int)rand() % n_polygons;
        triangleAreaAndNormal(meshcloud->points[mesh.polygons[i].vertices[0]], meshcloud->points[mesh.polygons[i].vertices[1]], meshcloud->points[mesh.polygons[i].vertices[2]], meshcloud->points[mesh.polygons[randint].vertices[0]], area, normal);
        poly_normals.push_back(normal);
        poly_areas.push_back(area);
    }
    // En este punto, "poly_normals" y "poly_areas" debieran tener tamaño "n_polygons".
    // Recorrer polígonos y crear patch para cada uno
    for (int i=0; i<n_polygons; i++){
        // Crear su grupo y añadirse a si mismo
        vector<int> patch;
        patch.push_back(i);
        double patch_area = poly_areas[i];
        // Añadir todos los parches de normal similar
        for (int j=0; j<n_polygons; j++){
            // Descartarse a si mismo
            if (i==j)
                continue;
            // Comparar diferencia de normal. Notar que ya vienen normalizadas
            double anglediff = acos(poly_normals[i].dot(poly_normals[j]));
            if (anglediff < PATCH_ANGLE_THRESHOLD){
                // Agregar al parche
                patch.push_back(j);
                patch_area += poly_areas[j];
            }
        }
        // Añadir grupo y area a arreglos externos
        patches.push_back(patch);
        areas.push_back(patch_area);
    }
    // En este punto, "patches" y "areas" son de tamaño "n_polygons"
    // Buscar area más grande
    double biggest_area = 0;
    int biggest_index = 0;
    for (int i=0; i<n_polygons; i++){
        if (areas[i] > biggest_area){
            biggest_area = areas[i];
            biggest_index = i;
        } 
    }
    // Retornar parche más grande
    // Por ahora lo dibujamos en verde
    for (int i=0; i<patches[biggest_index].size(); i++){
        stringstream surface_name, wire_name;
        surface_name << "parche_" << i;
        wire_name << "parche_" << i << "wire";
        drawPolygon(mesh, mesh.polygons[patches[biggest_index][i]],FLAT_SURFACE_COLOR[0], FLAT_SURFACE_COLOR[1], FLAT_SURFACE_COLOR[2], surface_name.str());
        drawPolygon(mesh, mesh.polygons[patches[biggest_index][i]],FLAT_SURFACE_WIRE_COLOR[0], FLAT_SURFACE_WIRE_COLOR[1], FLAT_SURFACE_WIRE_COLOR[2], wire_name.str(), false);
        // Engrosar línea de borde de polígono
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, FLAT_SURFACE_WIRE_WIDTH, wire_name.str(), 0);
    }
}

int main(int argc, char **argv){
    if (argc < 2){
        printf("Error: Debe especificar un archivo .pcd\n");
        exit(1);
    }
    if (argc == 3){
        // Se cae solo si entrega valor inválido
        PATCH_ANGLE_THRESHOLD = atof(argv[2]);
    }
    // Cargar nube de puntos
    PointCloud::Ptr cloud (new PointCloud);//, cloud_filt (new PointCloud);
    pcl::io::loadPCDFile(argv[1],*cloud);
    // Crear visualizador
    viewer = meshVis();
    // Visualizar nube de puntos
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "pointcloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "pointcloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "pointcloud");

    // Crear convex hull alrededor de nube
    pcl::PolygonMesh hull = getConvexHull(cloud);
    // Visualizar convex hull como wireframe
    viewer->addPolygonMesh (hull, "hull",0);
    viewer->setRepresentationToWireframeForAllActors();
    
    // Visualizar normales
    drawPolygonMeshNormals(hull);
    viewer->addText("Normales", LEFT_MARGIN, BOTTOM_MARGIN+(FONT_SIZE+1)*3, FONT_SIZE, CT_COLOR[0], CT_COLOR[1], CT_COLOR[2], "ct_text", 0);


    /*// Obtener polígono más grande y su normal
    pcl::Vertices biggest;
    Eigen::Vector3f b_normal;
    double b_area;
    biggestPolygon(hull, biggest, b_normal, b_area);

    //printf("Normal: (%f, %f, %f)\n", b_normal[0], b_normal[1], b_normal[2]);

    // Visualizar polígono
    drawPolygon(hull, biggest, 1.0, 0.0, 0.0, "biggest_polygon");

    // Obtener centro de masa
    pcl::PointXYZ ct = getCentroid(hull), cm = getCenterOfMass(hull, b_area);
    // Mostrar centroide
    drawPoint(ct, "Centroide", CT_COLOR[0], CT_COLOR[1], CT_COLOR[2]);
    // Mostrar centro de masa
    drawPoint(cm, "Centro de Masa", CM_COLOR[0], CM_COLOR[1], CM_COLOR[2]);
    // Proyectar ambos sobre plano del polígono
    pcl::PointXYZ ct_projected = projectPointOverPolygon(ct, b_normal, biggest, hull);
    pcl::PointXYZ cm_projected = projectPointOverPolygon(cm, b_normal, biggest, hull);
    // Mostrar ambas proyecciones
    drawPoint(ct_projected, "ct proyectado", CT_COLOR[0]+(1.0-CT_COLOR[0])*LIGHT_FACTOR, CT_COLOR[1]+(1.0-CT_COLOR[1])*LIGHT_FACTOR, CT_COLOR[2]+(1.0-CT_COLOR[2])*LIGHT_FACTOR);
    drawPoint(cm_projected, "cm proyectado", CM_COLOR[0]+(1.0-CM_COLOR[0])*LIGHT_FACTOR, CM_COLOR[1]+(1.0-CM_COLOR[1])*LIGHT_FACTOR, CM_COLOR[2]+(1.0-CM_COLOR[2])*LIGHT_FACTOR);
    // Mostrar texto explicativo
    viewer->addText("Centroide", LEFT_MARGIN, BOTTOM_MARGIN, FONT_SIZE, CT_COLOR[0], CT_COLOR[1], CT_COLOR[2], "ct_text", 0);
    viewer->addText("Centro de masa", LEFT_MARGIN, BOTTOM_MARGIN+FONT_SIZE+1, FONT_SIZE, CM_COLOR[0], CM_COLOR[1], CM_COLOR[2], "cm_text", 0);
    stringstream thres_text;
    thres_text << "Threshold: " << PATCH_ANGLE_THRESHOLD << " rad";
    viewer->addText(thres_text.str(), LEFT_MARGIN, BOTTOM_MARGIN+(FONT_SIZE+1)*2, FONT_SIZE, 1, 1, 1, "thres_text", 0);
    // Ver posición de las proyecciones
    printf("Centroide está %s del polígono\n", (pointInPolygon(ct_projected, biggest, hull) ? "DENTRO":"FUERA"));
    printf("Centro de Masa está %s del polígono\n", (pointInPolygon(cm_projected, biggest, hull) ? "DENTRO":"FUERA"));
    // Ver parche plano más grande
    biggestFlatPatch(hull);*/
    while (!viewer->wasStopped()){
        viewer->spinOnce(100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}