/*Algoritmo:

-Obtener nube de puntos del kinect
-Submuestrearlo
-iterar mientras los puntos que nos queden no bajen de un umbral (u otra condición, por ejemplo, hasta encontrar N modelos)
    -Segmentar
    -Obtener inliers de la segmentación
    -Extraerlos
    -Publicarlos

Dos opciones para separar los inliers de cada modelo:
    1) ESCOGIDA: definir distintos tópicos
    2) No extraerlos, sino que pintarlos
*/
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// Operaciones Matemáticas
#include <math.h>
// Para downsampling
#include <pcl/filters/voxel_grid.h>
// Para trabajar directamente con PointCloud<T> desde y hacia ROS
#include <pcl_ros/point_cloud.h>
// Para usar métodos de segmentación
#include <pcl/segmentation/sac_segmentation.h>
// Para extracción de índices
#include <pcl/filters/extract_indices.h>
// Para noción del tiempo
#include <sys/time.h>
#define NaN 10
#define DEBUG 1
// Constantes fijadas por mí
float leafsize = 0.05; // partición de downsampling
float segthres = 0.01; // umbral de distancia para ser inlier
// Reescritura del tipo
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
// Publisher
ros::Publisher plano1,plano2;

// Para mostrar tiempos
#define ROJO        "\x1b[31m"
#define VERDE       "\x1b[32m"
#define AMARILLO    "\x1b[33m"
#define AZUL        "\x1b[34m"
#define MAGENTA     "\x1b[35m"
#define CYAN        "\x1b[36m"
#define NO_COLOR    "\x1b[0m"
struct timeval tstart,t0,t1;
float timediff(struct timeval tf, struct timeval ti){
    long int tfms,tims;
    tfms=tf.tv_sec*1000+tf.tv_usec/1000.0;
    tims=ti.tv_sec*1000+ti.tv_usec/1000.0;
    return (float)((tfms-tims)/1000.0);
}
float timenow(){
    struct timeval ahora;
    gettimeofday(&ahora,NULL);
    return timediff(ahora,tstart);
}
/*Ojo: si quiero extraer ángulo entre vectores, recordar que se define como:
ángulo opuesto a lado ||a-b|| en un triángulo definido por lados ||a-b||, ||a|| y ||b||*/
float anguloVectores(float v1[3],float v2[3]){
    //Calcular ||v1|| y ||v2||
    // ||v1||^2
    double v1mod2=v1[0]*v1[0] + v1[1]*v1[1] + v1[2]*v1[2];
    // ||v2||^2
    double v2mod2=v2[0]*v2[0] + v2[1]*v2[1] + v2[2]*v2[2];
    if (v1mod2 == 0 or v2mod2 == 0){
        //undefined
        return (float)NaN;
    }
    double v1v2[3];
    v1v2[0]=v1[0]-v2[0];
    v1v2[1]=v1[1]-v2[1];
    v1v2[2]=v1[2]-v2[2];
    // ||v1-v2||^2
    double v1v2mod2=v1v2[0]*v1v2[0] + v1v2[1]*v1v2[1] + v1v2[2]*v1v2[2];
    double angulo=acos((v1mod2+v2mod2-v1v2mod2)/(2*sqrt(v1mod2*v2mod2)));
    return (float)angulo;
}
void kinect_cb(const PointCloud::ConstPtr& nube_in){
    // Para medir tiempo de cómputo:
    gettimeofday(&t0,NULL);
    if (DEBUG) printf(AZUL"DEBUG [%f]:" NO_COLOR " Nube recibida. Submuestreando...\n",timenow());
    // Submuestrear
    int npuntos_original = (int)nube_in->size();
    PointCloud::Ptr nube_downsampled(new PointCloud); // Los smart pointers hay que inicializarlos
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (nube_in);
    sor.setLeafSize (leafsize, leafsize, leafsize);
    sor.filter (*nube_downsampled);
    // Capturar cantidad de puntos originalmente
    int npuntos = (int)nube_downsampled->size();
    if (DEBUG) printf(AZUL"DEBUG [%f]:" NO_COLOR " Se redujo puntos al %f%% del original\n",timenow(),(100.0*npuntos/npuntos_original));
    // Crear elementos de segmentación
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE); //Modelo: Plano
    seg.setMethodType (pcl::SAC_RANSAC);    //Algoritmo: RANSAC
    seg.setDistanceThreshold (segthres);    //Threshold
    //opcional
    //seg.setMaxIterations (1000);
    PointCloud::Ptr nube_modelo (new PointCloud), nube_restante (new PointCloud);
    pcl::ModelCoefficients::Ptr coefs (new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZ> extractor;
    int nmodelos=0;
    int nmodelos_max=2;
    while (nmodelos<nmodelos_max){
        if (DEBUG) printf(AZUL"DEBUG [%f]:" NO_COLOR " Segmentando modelo n°%d...\n",timenow(),(nmodelos+1));
        seg.setInputCloud (nube_downsampled); // makeShared crea un smart pointer. Ojo que se debe usar con nubes VACÍAS.
        seg.segment (*inliers, *coefs);     // Hacer la segmentación y almacenar inliers y coeficientes.
        if (inliers->indices.size() == 0){
            printf("No pude encontrar ningún modelo!\n");
            break;
        }
        nmodelos++;
        // extraer inliers al modelo
        if (DEBUG) printf(AZUL"DEBUG [%f]:" NO_COLOR " Recortando puntos en el modelo (inliers)...\n",timenow());
        extractor.setInputCloud(nube_downsampled);
        extractor.setIndices(inliers);
        extractor.setNegative(false);
        extractor.filter(*nube_modelo);
        // publicar puntos
        switch (nmodelos){
            case 1:
                gettimeofday(&t1,NULL);
                if (DEBUG) printf(AZUL"DEBUG [%f]:" NO_COLOR " Tiempo hasta computar plano 1: %fs. Publicando.\n",timenow(),timediff(t1,t0));
                plano1.publish(nube_modelo);
                break;
            case 2:
                gettimeofday(&t1,NULL);
                if (DEBUG) printf(AZUL"DEBUG [%f]:" NO_COLOR " Tiempo hasta computar plano 2: %fs. Publicando.\n",timenow(),timediff(t1,t0));
                plano2.publish(nube_modelo);
                break;
        }
        // restar puntos ya encontrados
        extractor.setNegative(true);
        extractor.filter(*nube_restante);
        nube_downsampled.swap(nube_restante);
        if (DEBUG) printf(AZUL"DEBUG [%f]:" NO_COLOR " %d puntos restantes tras recorte (de %d)\n",timenow(),(int)nube_downsampled->size(),npuntos);
    }
    if (nmodelos == nmodelos_max){
        gettimeofday(&t1,NULL);
        if (DEBUG) printf(AZUL"DEBUG [%f]:" CYAN " ***%d modelos publicados*** "NO_COLOR". Demora total: "AMARILLO"%f"NO_COLOR"s,\nEsperando publicaciones del kinect...\n",timenow(),nmodelos,timediff(t1,t0));
    }
}

int main (int argc, char** argv){
    gettimeofday(&tstart,NULL);
    if (DEBUG) printf(AZUL"DEBUG [%f]:" NO_COLOR " Iniciando programa\n",timenow());
    ros::init (argc, argv, "segmentador");
    ros::NodeHandle nh;
    // Suscribirse al kinect 
    if (DEBUG) printf(AZUL"DEBUG [%f]:" NO_COLOR " Suscribiéndose a head_mount_kinect/depth/points\n",timenow());
    ros::Subscriber sub = nh.subscribe<PointCloud>("head_mount_kinect/depth/points", 1, kinect_cb);
    // Publisher. Entregará puntos que se ajustan al modelo (un plano).
    if (DEBUG) printf(AZUL"DEBUG [%f]:" NO_COLOR " Creando tópicos plano1_segmentado y plano2_segmentado\n",timenow());
    plano1 = nh.advertise<PointCloud> ("plano1_segmentado", 1);
    plano2 = nh.advertise<PointCloud> ("plano2_segmentado", 1);
    // Spin
    if (DEBUG) printf(AZUL"DEBUG [%f]:" NO_COLOR " Esperando publicaciones del kinect...\n",timenow());
    ros::spin ();
}