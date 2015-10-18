#include <csignal>
#include <ros/ros.h>
#include <memoria/GoToPose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

const float PI = 3.1415;
memoria::GoToPose gotopose_srv;

void signalHandler( int signum ){
    ros::shutdown();
    exit (EXIT_SUCCESS);
}
int main(int argc, char **argv){
    ros::init(argc, argv, "gotoposetest");
    ros::NodeHandle nh;
    ros::ServiceClient gotopose_client = nh.serviceClient<memoria::GoToPose>("go_to_pose");
    signal(SIGINT, signalHandler);

    // Enviar cosas al basedriver
    bool run=true;
    float x = (argc > 1? atof(argv[1]):0), y=(argc>2?atof(argv[2]):0), angle = (argc>3 ? PI*atof(argv[3])/180.0:0);
    // Crear pose
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = (argc > 4 ? argv[4] : "base_footprint");
    pose.pose.position.x=x;
    pose.pose.position.y=y;
    pose.pose.position.z=0;
    // Crear orientación a partir del ángulo
    tf::Vector3 zaxis(0,0,1);
    tf::Quaternion tf_orientation (zaxis, angle);
    tf::quaternionTFToMsg(tf_orientation,pose.pose.orientation);
    // Enviar pose
    gotopose_srv.request.pose = pose;
    if (not gotopose_client.call(gotopose_srv)){
        ROS_ERROR("Error al llamar al go_to_pose");
    }
    ROS_INFO("Error retornado: %s",gotopose_srv.response.error.what.c_str());
    return 0;
}