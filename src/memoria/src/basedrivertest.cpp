#include <ros/ros.h>
#include <memoria/BaseDriver.h>
#include <csignal>

const float PI = 3.1415;
memoria::BaseDriver basedriver_srv;

void signalHandler( int signum ){
    ros::shutdown();
    exit (EXIT_SUCCESS);
}
int main(int argc, char **argv){
    ros::init(argc, argv, "basedrivertest");
    ros::NodeHandle nh;
    ros::ServiceClient basedriver_client = nh.serviceClient<memoria::BaseDriver>("base_driver");
    signal(SIGINT, signalHandler);

    // Enviar cosas al basedriver
    bool run=true;
    float distance = atof(argv[1]), angle = atof(argv[2]);
    // while (run){
        basedriver_srv.request.distance = distance;
        basedriver_srv.request.angle = angle;
        ROS_INFO("Llamando basedriver con distance = %f y angle = %f", distance, angle);
        if (not basedriver_client.call(basedriver_srv)){
            ROS_INFO("Error al llamar al basedriver");
        }
        ROS_INFO("Error retornado: %s",basedriver_srv.response.error.what.c_str());
    // }
    return 0;
}