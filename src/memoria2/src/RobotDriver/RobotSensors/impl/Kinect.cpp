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
	ROS_DEBUG("KINECT: Me pidieron una nube. Recibiendo...");
	// Recibir nube, guardar y mandar signal. Si getNewScan la esperaba, la escuchará y actuará acorde a como corresponda.
	ROS_DEBUG("KINECT: Frame: %s", in_cloud->header.frame_id.c_str());
	*last_cloud = *in_cloud;
	last_cloud->header.frame_id = in_cloud->header.frame_id;
	cloud_received = true;
}
// Constructor y Destructor
Kinect::Kinect(){
	ROS_DEBUG("KINECT: Creando nodehandle");
	this->nh_ = new ros::NodeHandle;
	ROS_DEBUG("KINECT: Subscribiendose a topico de kinect");
	this->cloud_sub_ = this->nh_->subscribe<PointCloud<PointXYZ> >(Util::KINECT_TOPIC, 1, kinectCallback);
	PointCloud<PointXYZ>::Ptr temp_cloud1 (new PointCloud<PointXYZ>()), temp_cloud2 (new PointCloud<PointXYZ>());
	this->last_cloud_ = temp_cloud1;
	last_cloud = temp_cloud2;
	ROS_DEBUG("KINECT: Listo para atender peticiones");
}
Kinect::~Kinect(){
	ROS_DEBUG("KINECT: Destruyendo nodehandle");
	delete this->nh_;
}
PointCloud<PointXYZ>::Ptr Kinect::getNewCloud(){
	ROS_DEBUG("KINECT: Esperando una nube de puntos");
	get_scan = true;
	// Esperar signal de kinectCallback;
	// ALERTA, ALERTA!!! BUSYWAITING! ALERTA!!!!!
	while (not cloud_received){
		ros::Duration(BUSYWAIT_DELAY).sleep();
		ros::spinOnce();
	}
	cloud_received = false;
	get_scan = false;
	// Remover nans
	// PointCloud<PointXYZ>::Ptr cloud_no_nans (new PointCloud<PointXYZ>());
	vector<int> index;
	removeNaNFromPointCloud(*last_cloud, *(this->last_cloud_), index);
	ROS_DEBUG("KINECT: Después de remover nans: %d puntos", (int)this->last_cloud_->points.size());
	// this->last_cloud_ = last_cloud;
	return last_cloud_;
}