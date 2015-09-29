#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// para trabajar directamente con pcl::PointCloud<T> en ROS
#include <pcl_ros/point_cloud.h>

/* Este nodo dibuja un plano */

// Reescritura del tipo
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main (int argc, char** argv){
    // Crear nodo de nombre “my_pcl_tutorial” (obligatorio)
    ros::init (argc, argv, "dibujador_de_planos");
    // Crear punto de acceso al sistema ROS (obligatorio)
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<PointCloud> ("plano_dibujado", 1);

    // Crear publisher para entregar el resultado de crear el plano
    // En ros, se ve como el tópico ”/output”.
    // Recordar que el segundo parámetro es el tamaño del buffer.
  
    // Creo contenedor PCL de nueva nube de puntos
    PointCloud::Ptr nube (new PointCloud);
    // Inicializo mi contenedor
    nube->header.frame_id = "base_footprint";
    nube->width = 100; //cantidad de puntos en total
    nube->height = 1; //dataset no-organizado
    nube->points.resize (nube->width * nube->height); // Defino tamaño del dataset

    // Agrego uno a uno los puntos que quiero crear. Quiero un plano de 10x10 en xy
    for (size_t i = 0; i < nube->points.size(); ++i){
        float factor=0.1;
        nube->points[i].x = (i%10)*factor   ;
        nube->points[i].y = (int)(i*factor)*factor;//((int)i%10)*factor;
        nube->points[i].z = 1.0;
        printf ("Nuevo punto: %f, %f, %f\n", nube->points[i].x,nube->points[i].y,nube->points[i].z);

        // otro método para adjuntar puntos fácilmente es: push_back() que añade un nuevo punto al final de la nube.
    }
    ros::Rate frecuencia(10);
    int id=0;
    // Loop para enviar mensajes
    while (ros::ok()){
        // Publico los resultados
        pub.publish(nube);
        // delay del loop
        ros::spinOnce();
        frecuencia.sleep();
        id++;
    }
}