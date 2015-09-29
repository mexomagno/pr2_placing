#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// Para usar métodos de downsampling
#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input){

    // Contenedores para nube original y filtrada
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; // Puntero, nube original 
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);          // ConstPtr a partir del anterior
    pcl::PCLPointCloud2 cloud_filtered;                   // Contenedor para resultado del filtrado. No puntero!!!

    // Convertir input de sensor_msgs a PCL. Fijarse que ambos son punteros a PointCloud2ConstPtr pero uno de ROS y el otro de PCL
    pcl_conversions::toPCL(*input, *cloud);

    // Filtrar
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloudPtr);
    sor.setLeafSize (0.1, 0.1, 0.1);
    // Resultado almacenado en cloud_filtered
    sor.filter (cloud_filtered);

    // Crear contenedor del resultado
    sensor_msgs::PointCloud2 output;
    // Convertir resultado de PCL a ROS. Notar que ambos son PointCloud2 pero uno de ROS y otro de PCL 
    pcl_conversions::fromPCL(cloud_filtered, output);

    // Enviar mensaje al tópico. Recibe el objeto mensaje, que debe ser el mismo entregado en el template de “advertise”
    pub.publish (output);
}

int main (int argc, char** argv){
    // Crear nodo con el nombre del paquete al que pertenece (obligatorio)
    ros::init (argc, argv, "tutoriales_pcl");
    // Crear punto de acceso al sistema ROS (obligatorio)
    ros::NodeHandle nh;

    // Objeto que nos suscribe al tópico “input” y define el handler de los datos.
    // El input se le puede entregar en la consola, mapeando un tópico real al 
    // nombre “input” como sigue: “input:=<cierto tópico pointcloud>”  

    ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

    // Crear publisher para entregar el resultado de operar sobre el input
    // En ros, se ve como el tópico ”/output”.
    // Recordar que el segundo parámetro es el tamaño del buffer.
    // La variable "pub" fue declarada globalmente para que el handler "cloud_cb" pueda usarla.
    pub = nh.advertise<sensor_msgs::PointCloud2> ("kinect_downsampleado", 1);

    // Spin
    ros::spin ();
}