#include <csignal>
#include <ros/ros.h>
#include <memoria/GripperDriver.h>

memoria::GripperDriver gripperdriver_srv;

void signalHandler( int signum ){
    ros::shutdown();
    exit (EXIT_SUCCESS);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "gripperdrivertest");
    ros::NodeHandle nh;
    ros::ServiceClient gripperdriver_client = nh.serviceClient<memoria::GripperDriver>("gripper_driver");
    signal(SIGINT, signalHandler);

    // Enviar cosas al GripperDriver
    gripperdriver_srv.request.opening = 1;
    gripperdriver_srv.request.right = false;
    ROS_INFO("Abriendo Gripper izquierdo");
    if (not gripperdriver_client.call(gripperdriver_srv)){
        ROS_ERROR("Error al comunicarse con 'gripper_driver'");
        exit(1);
    }
    ROS_INFO("Abriendo Gripper derecho");
    gripperdriver_srv.request.right = true;
    if (not gripperdriver_client.call(gripperdriver_srv)){
        ROS_ERROR("Error al comunicarse con 'gripper_driver'");
        exit(1);
    }
    ROS_INFO("Cerrando Gripper izquierdo");
    gripperdriver_srv.request.opening = 0;
    gripperdriver_srv.request.right = false;
    if (not gripperdriver_client.call(gripperdriver_srv)){
        ROS_ERROR("Error al comunicarse con 'gripper_driver'");
        exit(1);
    }
    ROS_INFO("Cerrando Gripper derecho");
    gripperdriver_srv.request.right = true;
    if (not gripperdriver_client.call(gripperdriver_srv)){
        ROS_ERROR("Error al comunicarse con 'gripper_driver'");
        exit(1);
    }
    exit(0);
}