#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// para poder usar cout
#include <iostream>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
// Para usar métodos de segmentación
#include <pcl/segmentation/sac_segmentation.h>
using namespace std;
ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input){
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud

    pcl::PointCloud<pcl::PointXYZ> cloud; // Contenedor de puntos, usando tipos de PCL
    pcl::fromROSMsg (*input, cloud);      // Convertir input y poner en contenedor

    // Crear objetos para coeficientes y para los inliers. Ojo que los inliers son PointIndices.
    pcl::ModelCoefficients coefficients;
    // Contenedor de índices de puntos que estén dentro de la segmentación
    pcl::PointIndices inliers;
    // Crear el objeto de segmentación, templateado con PointXYZ
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Opcional
    seg.setOptimizeCoefficients (true);
    // Obligatorio
    seg.setModelType (pcl::SACMODEL_PLANE); //Modelo: Plano
    seg.setMethodType (pcl::SAC_RANSAC);    //Algoritmo: RANSAC
    seg.setDistanceThreshold (0.01);        //Threshold: 0.01 m
    //opcional
    //seg.setMaxIterations (1000);
    seg.setInputCloud (cloud.makeShared ()); // El makeshared crea un shaped_ptr de Boost (librería) para "cloud".
    seg.segment (inliers, coefficients);     // Hacer la segmentación y almacenar inliers y coeficientes.
    printf ("Coeficientes: %f, %f, %f, %f\n", (&coefficients)->values[0],(&coefficients)->values[1],(&coefficients)->values[2],(&coefficients)->values[3]);
    std::cout << "Frame: " << cloud.header.frame_id << std::endl;
    // o se puede convertir el string std:string a char * usando c_str().
    // Publicar resultados
    pcl_msgs::ModelCoefficients ros_coefficients; // Contenedor de coeficientes ROS
    pcl_conversions::fromPCL(coefficients, ros_coefficients); // Convertir de PCL a ROS. Notar que ambos son ModelCoefficients, pero uno es pcl:: y el otro es pcl_msgs:: (ROS).
    pub.publish (ros_coefficients);
}

int main (int argc, char** argv){
    // Crear nodo de nombre “my_pcl_tutorial” (obligatorio)
    ros::init (argc, argv, "my_pcl_tutorial");
    // Crear punto de acceso al sistema ROS (obligatorio)
    ros::NodeHandle nh;

    // Objeto que nos suscribe al tópico “input” y define el handler de los datos.
    // El input se le puede entregar en la consola, mapeando un tópico real al 
    // nombre “input” como sigue: “input:=<cierto tópico pointcloud>”  

    ros::Subscriber sub = nh.subscribe ("head_mount_kinect/depth/points", 1, cloud_cb);

    // Crear publisher para entregar el resultado de operar sobre el input
    // En ros, se ve como el tópico ”/output”.
    // Recordar que el segundo parámetro es el tamaño del buffer.
    // La variable "pub" fue declarada globalmente para que el handler "cloud_cb" pueda usarla.
    pub = nh.advertise<pcl_msgs::ModelCoefficients> ("coeficientes_plano_segmentado", 1);

    // Spin
    ros::spin ();
}