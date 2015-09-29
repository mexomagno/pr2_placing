#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// para trabajar directamente con pcl::PointCloud<T> en ROS
#include <pcl_ros/point_cloud.h>

/* Este nodo intenta ser capaz de tomar cuatro coeficientes de tipo ModelCoefficients y crear una nube de puntos
que muestren el plano.
Debe estar ejecutándose "planar_segmentation". */

// Reescritura del tipo
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
ros::Publisher pub;

void coeff_cb (const pcl_msgs::ModelCoefficientsConstPtr& input){
    // Creo contenedor PCL de nueva nube de puntos
    PointCloud::Ptr nube (new PointCloud);
    // Inicializo mi contenedor
    // Ojo con este frame_id: Fue obtenido empíricamente, leyendo el frame de la nube de entrada. Sólo así los puntos tienen sentido.
    nube->header.frame_id = "head_mount_kinect_ir_optical_frame";
    nube->width = 100; //cantidad de puntos en total
    nube->height = 1; //dataset no-organizado
    nube->points.resize (nube->width * nube->height); // Defino tamaño del dataset

    // Agrego uno a uno los puntos que quiero crear. Quiero un plano de 10x10 que cumpla la ecuación Ax+By+Cz+D=0 (z=[Ax+By+D]/-C)
    // Los coeficientes vienen de los datos a los que estoy suscrito, osea, del input
    printf ("Coeficientes: %f, %f, %f, %f\n", input->values[0],input->values[1],input->values[2],input->values[3]);
    for (size_t i = 0; i < nube->points.size(); ++i){
        float factor=0.1;
        nube->points[i].x = (i%10)*factor   ;
        nube->points[i].y = (int)(i*factor)*factor;
        nube->points[i].z = (input->values[0]*nube->points[i].x+input->values[1]*nube->points[i].y+input->values[3])/(-1*input->values[2]);
        printf ("Nuevo punto: %f, %f, %f\n", nube->points[i].x,nube->points[i].y,nube->points[i].z);
        // otro método para adjuntar puntos fácilmente es: push_back() que añade un nuevo punto al final de la nube.
    }
    // Publico los resultados
    pub.publish(nube);
}
int main (int argc, char** argv){
    // Crear nodo con el nombre del paquete al que pertenece (obligatorio)
    ros::init (argc, argv, "plano_desde_coeficientes");
    // Crear punto de acceso al sistema ROS (obligatorio)
    ros::NodeHandle nh;

    // Suscribirse a topic de coeficientes de segmentación
    ros::Subscriber sub = nh.subscribe("coeficientes_plano_segmentado", 1, coeff_cb);

    // Crear publisher para entregar plano
    pub = nh.advertise<PointCloud>("plano_dibujado2", 1);

    // Spin
    ros::spin ();
}