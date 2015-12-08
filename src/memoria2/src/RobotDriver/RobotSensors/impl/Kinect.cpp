#include "../Kinect.h"

// VARIABLES GLOBALES
bool get_scan = false;
bool cloud_received = false;
PointCloud<PointXYZ>::Ptr last_cloud;
const float BUSYWAIT_DELAY = 0.01;

// MÉTODOS
void kinectCallback(const PointCloud<PointXYZ>::ConstPtr &in_cloud){
	if (not get_scan)
		return;
	ROS_DEBUG("Kinect: Me pidieron una nube. Recibiendo...");
	// Recibir nube, guardar y mandar signal. Si getNewScan la esperaba, la escuchará y actuará acorde a como corresponda.
	*last_cloud = *in_cloud;
	cloud_received = true;
}
// Constructor y Destructor
Kinect::Kinect(){
	ROS_DEBUG("Kinect: Creando nodehandle");
	this->nh_ = new ros::NodeHandle;
	ROS_DEBUG("Kinect: Subscribiendose a topico de kinect");
	this->cloud_sub_ = this->nh_->subscribe<PointCloud<PointXYZ> >(Util::KINECT_TOPIC_SELF_FILTERED, 1, kinectCallback);
	PointCloud<PointXYZ>::Ptr temp_cloud1 (new PointCloud<PointXYZ>()), temp_cloud2 (new PointCloud<PointXYZ>());
	this->last_cloud_ = temp_cloud1;
	last_cloud = temp_cloud2;
	ROS_DEBUG("Kinect: Listo para atender peticiones");
}
Kinect::~Kinect(){
	ROS_DEBUG("Kinect: Destruyendo nodehandle");
	delete this->nh_;
}
PointCloud<PointXYZ>::Ptr Kinect::getNewCloud(){
	ROS_DEBUG("Kinect: Esperando una nube de puntos");
	get_scan = true;
	// Esperar signal de kinectCallback;
	// ALERTA, ALERTA!!! BUSYWAITING! ALERTA!!!!!
	while (not cloud_received){
		ros::Duration(BUSYWAIT_DELAY).sleep();
		ros::spinOnce();
	}
	cloud_received = false;
	get_scan = false;
	last_cloud_ = last_cloud;
	return last_cloud_;
}