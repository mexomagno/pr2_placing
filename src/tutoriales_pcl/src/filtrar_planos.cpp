#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// Para downsampling
#include <pcl/filters/voxel_grid.h>
// Para trabajar directamente con PointCloud<T> desde y hacia ROS
#include <pcl_ros/point_cloud.h>
// Para usar métodos de segmentación
#include <pcl/segmentation/sac_segmentation.h>
// Para extracción de índices
#include <pcl/filters/extract_indices.h>
// Transformaciones de nube de puntos
#include <pcl_ros/transforms.h>
// Centroide
#include <pcl/common/centroid.h>
// orientar normales hacia viewpoint
#include <pcl/features/normal_3d.h>
// Para publicar poses
#include <geometry_msgs/PoseStamped.h>
// Para tf
#include <tf/transform_listener.h>
// Operaciones Matemáticas
#include <math.h>
// para poder usar cout
#include <iostream>
// strings (concatenación con +)
#include <string>

/*Este programa tomará una nube de puntos adquirida por el kinect, y será capaz de filtrar planos en la imagen bajo los siguientes criterios:
    - Altura del plano relativa al suelo
    - Ángulo del plano relativo al suelo
El resultado es un plano con todos los puntos que caen dentro de su modelo. */
/* Algoritmo propuesto:

- Recibir nube de puntos
- Downsamplear (opcional)
- Iterar
    - Segmentar planos
    - Verificar criterios
    -   Si se cumplen, Publicar y terminar iteración
    -   Si no, eliminar puntos de la nube y continuar buscando una cantidad limitada de veces

Datos importantes:
La nube de puntos del kinect tiene como frame central "head_mount_kinect_ir_link".
Para verificar los criterios, necesitamos una noción del frame del suelo (centro del robot).
Entonces, necesitamos obtener la transformación necesaria entre un frame y otro, y aplicarla de alguna de estas dos maneras:
1) [ESCOGIDA] Transformar el plano encontrado a coordenadas del SUELO, y aplicar criterios directamente
2) "Transformar criterios" a coordenadas del kinect y aplicar.
La implementación actual considera el primer caso.

Ojo que la transformación se puede hacer de dos maneras:
1) Transformar todos los puntos de la nube al frame de la base y obtener ahí el plano.
En este caso, habría que modificar el frame_id de la nube publicada. Simple, entendible, pero ineficiente.
2) [ESCOGIDA] Transformar sólo la "normal" del plano encontrado al frame base.

*/
// Mis constantes
float LEAFSIZE = 0.05; // partición de downsampling
float SEG_THRESHOLD = 0.01; // umbral de distancia para ser inlier
int MAX_ITERATIONS = 2; //10
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

using namespace std;
ros::Publisher plano1,plano2,normal1,normal2;
int  mierda=0;

float toRad(int grados){
    return grados*2*3.1415/360.0;
}
float toGrad(float rad){
    return rad*360/(2*3.1415);
}
geometry_msgs::Quaternion coefsToQuaternionMsg(float a, float b, float c){
    float vmod=sqrt(a*a+b*b+c*c);
    float pitch = -(1.0*asin(1.0*c/vmod));
    float yaw = (a==0?toRad(90):atan(b/a)) + (a>=0?0:toRad(180));
    return tf::createQuaternionMsgFromRollPitchYaw(0,pitch,yaw);
}
void kinect_cb(const PointCloud::ConstPtr& nube_in){
/*    tf::TransformListener tfl;
    tf::StampedTransform transf;*/
    // Submuestrear
    int npuntos = (int)nube_in->size();
    PointCloud::Ptr nube_submuestreada(new PointCloud), nube_inliers(new PointCloud);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (nube_in);
    sor.setLeafSize (LEAFSIZE, LEAFSIZE, LEAFSIZE);
    sor.filter (*nube_submuestreada);
    int npuntos_submuestreada = (int)nube_submuestreada->size();
    // Crear elementos de segmentación
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE); //Modelo: Plano
    seg.setMethodType (pcl::SAC_RANSAC);    //Algoritmo: RANSAC
    seg.setDistanceThreshold (SEG_THRESHOLD);    //Threshold
    //opcional
    //seg.setMaxIterations (1000);
    PointCloud::Ptr plano_candidato (new PointCloud), nube_restante (new PointCloud);
    pcl::ModelCoefficients::Ptr coefs (new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZ> extractor;
    bool found=false;
// ***********************************************



    // obtener transform
  /*  try{
        tfl.waitForTransform("/"+nube_in->header.frame_id, "/odom_combined", ros::Time(0),ros::Duration(10.0));
        tfl.lookupTransform("/"+nube_in->header.frame_id, "/odom_combined",ros::Time(0), transf);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s\n",ex.what());
      ros::Duration(1.0).sleep();
    }
    std::cout << "child_frame_id_=" << transf.child_frame_id_ << ", frame_id_:" << transf.frame_id_ << std::endl;
    printf("getOrigin entrega: x:%f, y:%f, z:%f\n", transf.getOrigin().x(),transf.getOrigin().y(),transf.getOrigin().z());
    */// mover la nube de puntos desde el kinect al origen
    //sensor_msgs::PointCloud2 input,output;
    //PointCloud output;
    //pcl::toROSMsg(*nube_submuestreada,input);
    //pcl_ros::transformPointCloud(*nube_submuestreada,*nube_submuestreada,transf);
    //pcl::fromROSMsg(output,*nube_submuestreada);

    // Iterar MAX_ITERATIONS veces. Puede que sea redundante.
    for (int i=1; i<=MAX_ITERATIONS;i++){
        // SEGMENTAR
        seg.setInputCloud (nube_submuestreada);
        seg.segment (*inliers, *coefs);
        if (inliers->indices.size() == 0){
            printf("Iteración %d: No pude encontrar ningún modelo! Abortando...\n", i);
            break;
        }
        found=true;
        //printf("Coeficientes: %f,%f,%f,%f\n",coefs->values[0],coefs->values[1],coefs->values[2],coefs->values[3]);
        // EXTRAER INLIERS
        extractor.setInputCloud(nube_submuestreada);
        extractor.setIndices(inliers);
        extractor.setNegative(false);
        extractor.filter(*nube_inliers);
        // CREAR MENSAJE CON LA NORMAL
        geometry_msgs::PoseStamped vn;
        vn.header.frame_id = nube_in->header.frame_id; // Perspectiva del pointcloud
        vn.header.stamp = ros::Time::now();
        // Posicionar en el centroide de los inliers
        Eigen::Vector4f centroide, eigennormal;
        compute3DCentroid(*nube_inliers,centroide);
        vn.pose.position.x = centroide(0);
        vn.pose.position.y = centroide(1);
        vn.pose.position.z = centroide(2);
        // Orientar normal
        eigennormal(0)=coefs->values[0];
        eigennormal(1)=coefs->values[1];
        eigennormal(2)=coefs->values[2];
        eigennormal(3)=0; // para poder normalizar
        eigennormal.normalize();
        pcl::PointXYZ pclnormal (centroide(0),centroide(1),centroide(2));
        pcl::flipNormalTowardsViewpoint(pclnormal,0,0,0,eigennormal); // asegurar orientación hacia la cámara
        vn.pose.orientation=coefsToQuaternionMsg(eigennormal(0),eigennormal(1),eigennormal(2));
        // PUBLICAR PLANO Y NORMAL
        switch (i){
            case 1:
                plano1.publish(nube_inliers);
                normal1.publish(vn);
                break;
            case 2:
                plano2.publish(nube_inliers);
                normal2.publish(vn);
                break;
        }
        // REDUCIR NUBE RESTANTE
        extractor.setNegative(true);
        extractor.filter(*nube_inliers);
        nube_submuestreada.swap(nube_inliers);

//************************************************

        // VERIFICAR CRITERIOS
        // Transportar normal al frame base
        // Verificar altura mínima (y máxima?) del plano
        // Verificar ángulo máximo con el suelo

        /*if (Cumple los criterios anteriores){
            // Extraer puntos del plano
            extractor.setInputCloud(nube_submuestreada);
            extractor.setIndices(inliers);
            extractor.setNegative(false);
            extractor.filter(*plano_candidato);
            // publicar plano
            plano.publish(plano_candidato);
            printf ("Iteración %d: Plano encontrado y publicado.\n",i);
            found=true;
            break;
        }*/
        // No cumple criterios anteriores: Seguir iterando
        // restar puntos ya encontrados
/*        extractor.setNegative(true);
        extractor.filter(*nube_restante);
        nube_submuestreada.swap(nube_restante);*/
    }
    if (not found) printf("En %d iteraciones NO se pudo encontrar un plano adecuado.\n",MAX_ITERATIONS);
    printf("Esperando siguiente nube de puntos...\n");
}
int main (int argc, char** argv){
    ros::init(argc, argv, "filtrar_plano");
    ros::NodeHandle nh;
    // Suscribirse al kinect
    ros::Subscriber sub = nh.subscribe<PointCloud>("head_mount_kinect/depth/points", 1, kinect_cb);
    // Publisher. Entregará puntos que se ajustan al modelo (un plano).
    plano1 = nh.advertise<PointCloud> ("plano_optimo_1", 1);
    plano2 = nh.advertise<PointCloud> ("plano_optimo_2", 1);
    normal1 = nh.advertise<geometry_msgs::PoseStamped> ("normal_plano_optimo_1", 1);
    normal2 = nh.advertise<geometry_msgs::PoseStamped> ("normal_plano_optimo_2", 1);
    ros::spin ();
}