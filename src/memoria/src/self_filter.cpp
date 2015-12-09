#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pr2_navigation_self_filter/self_see_filter.h>

using namespace pcl;
using namespace std;

const string KINECT_TOPIC = "/head_mount_kinect/depth_registered/points";

ros::Subscriber kinect_sub;
ros::Publisher kinect_pub;
filters::SelfFilter<PointXYZ> *filtro;

void kinectCallback(const PointCloud<PointXYZ>::ConstPtr &in_cloud){
	ROS_INFO("Recibo una nube de puntos");
	// filtrar robot
	PointCloud<PointXYZ>::Ptr out_cloud(new PointCloud<PointXYZ>());
	
	// Republicar nube
	*out_cloud = *in_cloud;
	kinect_pub.publish(*out_cloud);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "self_filter_y_wea");
	ros::NodeHandle nh;
	// Subscribirse a kinect
	kinect_sub = nh.subscribe<PointCloud<PointXYZ> >(KINECT_TOPIC, 1, kinectCallback);
	// Publicador filtrado
	kinect_pub = nh.advertise<PointCloud<PointXYZ> >("kinect_filtrada", 1);
	// Crear filtro
	filtro = new filters::SelfFilter<PointXYZ>(nh);
	ros::spin();
}