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
using namespace std;
// Mis constantes
const float LEAFSIZE = 0.05; // partición de downsampling
const float SEG_THRESHOLD = 0.01; // umbral de distancia para ser inlier
const int MAX_ITERATIONS = 3; //10
const float MIN_HEIGHT = 0.2; // umbral de altura mínima de un mesón
const float PITCH_THRESHOLD = 0.035; // delta de radianes de inclinación
const float DESIRED_PITCH = -3.14159/2.0; // Inclinación deseada del plano
const float WAIT_TRANSFORM_TIMEOUT = 1.0;
const std::string FIXED_FRAME = "/odom_combined";
const bool DEBUG = true;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// ******* Mediciones de tiempo
#define ROJO        "\x1b[31m"
#define VERDE       "\x1b[32m"
#define AMARILLO    "\x1b[33m"
#define AZUL        "\x1b[37m"//"\x1b[34m"
#define MORADO      "\x1b[35m"
#define CYAN        "\x1b[36m"
#define BLANCO      "\x1b[37m"
#define NO_COLOR    "\x1b[0m"
struct timeval tstart,t0,t1,taux;
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
// ******* FIN mediciones de tiempo

// ******* Simples conversores *******
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
// ******* FIN simples conversores *******

ros::Publisher plano, normal_odom;//, normal;
void kinect_cb(const PointCloud::ConstPtr& nube_in){
    if (DEBUG) printf(AZUL"DEBUG [%f]: "AMARILLO"Nube recibida\n",timenow());
    gettimeofday(&t0,NULL);
    // Obtener transformación de este momento
    tf::TransformListener transf_l;
    tf::StampedTransform transf;
    ros::Time transf_time = ros::Time(0);//ros::Time(nube_in->header.stamp/1000000.0);//ros::Time(0); // Debiera ser el tiempo de cuando llegó la nube
    const std::string   target_frame="/"+nube_in->header.frame_id, // "/head_mount_kinect_ir_..."
                        source_frame=FIXED_FRAME;
    try{
        if (DEBUG) printf(AZUL"DEBUG [%f]: Esperando un transform actualizado...\n"NO_COLOR,timenow());
        if ( not transf_l.waitForTransform(target_frame, source_frame, transf_time, ros::Duration(WAIT_TRANSFORM_TIMEOUT))){
            printf("\nNo se pudo encontrar una transformación decente\n"AMARILLO"Esperando siguiente nube de puntos...\n");
            return;
        }
        if (DEBUG) printf(AZUL"DEBUG [%f]: "VERDE"Encontrado!\n",timenow());
        // almacenar transformación
        transf_l.lookupTransform(target_frame, source_frame,ros::Time(0), transf);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("Mierda, Error!: %s\n"AMARILLO"Esperando siguiente nube de puntos...\n",ex.what());
        return;
        //ros::Duration(1.0).sleep();
    }
    cout << "Transf: " << transf.stamp_ << ", nube_in: " << nube_in->header.stamp << endl;
    gettimeofday(&taux,NULL);
    float time_transform=timediff(taux,t0);
    if (DEBUG) printf(AZUL"DEBUG [%f]: "NO_COLOR"Submuestreando... (leafsize = %f) ",timenow(),LEAFSIZE);
    // ************** Submuestrear ***************
    int npuntos = (int)nube_in->size();
    PointCloud::Ptr nube_submuestreada(new PointCloud), nube_inliers(new PointCloud);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (nube_in);
    sor.setLeafSize (LEAFSIZE, LEAFSIZE, LEAFSIZE);
    sor.filter (*nube_submuestreada);
    int npuntos_submuestreada = (int)nube_submuestreada->size();
    if (DEBUG) printf(VERDE"OK"NO_COLOR". N° de puntos resultante: %d de %d (%f%%)\n",npuntos_submuestreada,npuntos,(npuntos_submuestreada*100.0/npuntos));
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
    // ************** Buscar mesa según criterios. Intentar MAX_ITERATIONS veces *****************
    for (int i=1; i<=MAX_ITERATIONS;i++){
        if (DEBUG) printf(AZUL"DEBUG [%f]: "CYAN"Iteración %d: "NO_COLOR"Segmentando...\n",timenow(),i);
        // SEGMENTAR
        seg.setInputCloud (nube_submuestreada);
        seg.segment (*inliers, *coefs);
        if (inliers->indices.size() == 0){
            if (DEBUG) printf(AZUL"DEBUG [%f]: "CYAN"Iteración %d: "ROJO"No pude encontrar ningún modelo! Abortando...\n",timenow(),i);
            break;
        }
        // ********* EXTRAER INLIERS **********
        extractor.setInputCloud(nube_submuestreada);
        extractor.setIndices(inliers);
        extractor.setNegative(false);
        extractor.filter(*nube_inliers);
        // ********* CREAR MENSAJE CON LA NORMAL **********
        geometry_msgs::PoseStamped vn; // Vector Normal
        vn.header.frame_id = nube_in->header.frame_id; // Perspectiva del pointcloud
        vn.header.stamp = ros::Time(nube_in->header.stamp/1000000.0);//stamp de PointCloud<T> a stamp de ROS
        // Ubicar pose en el centroide de los inliers
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
        pcl::flipNormalTowardsViewpoint(pclnormal,0,0,0,eigennormal); // Importante y notable: asegurar orientación hacia la cámara
        vn.pose.orientation=coefsToQuaternionMsg(eigennormal(0),eigennormal(1),eigennormal(2));
        if (DEBUG) {
            printf(AZUL"DEBUG [%f]: "CYAN"Iteración %d:"NO_COLOR" Transformando vector normal de "MORADO,timenow(),i);
            std::cout << target_frame;
            printf(NO_COLOR" a "MORADO);
            std::cout << source_frame;
            if (DEBUG) printf(NO_COLOR"...\n");
        }
        // crear normal transportada
        geometry_msgs::PoseStamped vn_odom;
        vn.header.stamp = transf.stamp_; // para que transformPose funcione!!!
        transf_l.transformPose(source_frame,vn,vn_odom);
        //printf("vn antes  : (x,y,z)=(%f,%f,%f) frame_id=",vn.pose.position.x,vn.pose.position.y,vn.pose.position.z);
        //std::cout << vn.header.frame_id << std::endl;
        //printf("vn despues: (x,y,z)=(%f,%f,%f) frame_id=",vn_odom.pose.position.x,vn_odom.pose.position.y,vn_odom.pose.position.z);
        //std::cout << vn_odom.header.frame_id << std::endl;
        // ********* VERIFICAR CRITERIOS *********
        // Osea:
        // 1) Verificar altura mínima (y máxima?) del plano
        // 2) Verificar ángulo máximo con el suelo (ángulo respecto a Z)
        tf::Quaternion quat;
        tf::quaternionMsgToTF(vn_odom.pose.orientation, quat);
        //double angle = quat.getAngle();
        //tf::Vector3 axis = quat.getAxis();
        //printf("angle= %f, ax=%f,ay=%f,az=%f\n",angle,axis.x(),axis.y(),axis.z());
        double roll,pitch,yaw;
        tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
        // Sólo nos interesa Pitch
        //printf("roll=%f,pitch=%f,yaw=%f\n",roll,pitch,yaw);
        if (DEBUG) printf(AZUL"DEBUG [%f]: "CYAN"Iteración %d: "NO_COLOR"Verificando que el plano cumpla los criterios...\n",timenow(),i);
        if (vn_odom.pose.position.z > MIN_HEIGHT and DESIRED_PITCH-PITCH_THRESHOLD < pitch and pitch < DESIRED_PITCH+PITCH_THRESHOLD){
            // DING DING DING! Tenemos al ganador!
            // Cumple criterios de aceptación
            if (DEBUG) printf(AZUL"DEBUG [%f]: "CYAN"Iteración %d: "VERDE"TENEMOS UN GANADOR\n",timenow(),i);
            printf("Plano cumple criterios porque z=%f y se inclina en %f°\n",vn_odom.pose.position.z,toGrad(pitch));
            // Extraer puntos del plano
            extractor.setInputCloud(nube_submuestreada);
            extractor.setIndices(inliers);
            extractor.setNegative(false);
            extractor.filter(*plano_candidato);
            if (DEBUG) printf(AZUL"DEBUG [%f]: "CYAN"Iteración %d: "NO_COLOR"Publicando plano y su normal...\n",timenow(),i);
            // publicar plano
            plano.publish(plano_candidato);
            // publicar normal
            normal_odom.publish(vn_odom);
            //printf ("Iteración %d: Plano encontrado y publicado.\n",i);
            found=true;
            break;
        }
        // No cumple criterios anteriores: Seguir iterando
        // restar puntos ya encontrados
        if (DEBUG) printf(AZUL"DEBUG [%f]: "CYAN"Iteración %d: "NO_COLOR"Plano "ROJO"no cumple"NO_COLOR" porque z=%f y se inclina en %f°\n",timenow(),i,vn_odom.pose.position.z,toGrad(pitch));
        extractor.setNegative(true);
        extractor.filter(*nube_restante);
        nube_submuestreada.swap(nube_restante);
    }
    if (not found) printf(ROJO"En %d iteraciones NO se pudo encontrar un plano adecuado.\n",MAX_ITERATIONS);
    gettimeofday(&t1,NULL);
    if (DEBUG) printf(BLANCO"Estadísticas: \n\tTamaño nube entrante: %d puntos\n\tTamaño nube procesada: %d puntos\n\tPorcentaje submuestreo: %f%%\n\tTiempo espera transform: %f s\n\tTiempo total invertido: %f s\n",npuntos,npuntos_submuestreada,(100-(npuntos_submuestreada*100.0/npuntos)),time_transform,timediff(t1,t0));
    if (DEBUG) printf(AZUL"DEBUG [%f]: "AMARILLO"Esperando siguiente nube de puntos...\n",timenow());
}

int main (int argc, char** argv){
    gettimeofday(&tstart,NULL);
    if (DEBUG) printf(AZUL"DEBUG [%f]: "NO_COLOR"Iniciando programa\n",timenow());
    ros::init(argc, argv, "filtrar_plano2");
    ros::NodeHandle nh;
    // Suscribirse al kinect
    ros::Subscriber sub = nh.subscribe<PointCloud>("head_mount_kinect/depth/points", 1, kinect_cb);
    // Publisher. Entregará puntos que se ajustan al modelo (un plano).
    plano = nh.advertise<PointCloud> ("plano_optimo", 1);
    // normal = nh.advertise<geometry_msgs::PoseStamped> ("normal_plano_optimo", 1);
    normal_odom = nh.advertise<geometry_msgs::PoseStamped> ("normal_plano_optimo_odom", 1);
    if (DEBUG) printf(AZUL"DEBUG [%f]:\n\tSuscripciones:\n\t\t/head_mount_kinect/depth/points\n\tPublicaciones:\n\t\t/plano_optimo\n\t\t/normal_plano_optimo_odom\n"NO_COLOR,timenow());
    ros::spin ();
}