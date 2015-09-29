
#include <ros/ros.h>
// Mensajes para Action Interface de la cabeza
#include <pr2_controllers_msgs/PointHeadGoal.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <geometry_msgs/Point.h>
// 
#include <actionlib/client/simple_action_client.h>

// typedef para simplificar las cosas
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

void lookAt(std::string frame_id, double x, double y, double z, PointHeadClient *phc){
    pr2_controllers_msgs::PointHeadGoal goal;
    // Construir el punto al que la cabeza va a mirar
    // Se expresará respecto al frame "base_footprint"
    geometry_msgs::Point punto;
    goal.target.header.frame_id = frame_id;
    punto.x = x;
    punto.y = y;
    punto.z = z;
    goal.target.point = punto;
    // Se quiere que el eje X de la cámara apunte al punto
    goal.pointing_frame = "high_def_frame"; // podría ser otro si se quisiera.
    goal.pointing_axis.x = 1;
    goal.pointing_axis.y = 0;
    goal.pointing_axis.z = 0;
    // Limitar velocidades
    goal.max_velocity = 2; // rad/s
    goal.min_duration = ros::Duration(0.5);
    phc->sendGoal(goal);
    if (not phc->waitForResult(ros::Duration(15)))
        ROS_INFO("Timeout antes de llegar a la pose final");
}
int main(int argc,char** argv){
    ros::init(argc, argv, "movimiento_basico");
    
    // Cliente de movimiento
    PointHeadClient *pointheadclient = new PointHeadClient("/head_traj_controller/point_head_action", true);
    // Esperar el server
    while (!pointheadclient->waitForServer(ros::Duration(5.0))){
        ROS_INFO("Esperando al point_head_action server...");
    }
    ROS_INFO("Mirando al frente");
    lookAt("base_footprint",5,0,0, pointheadclient);
    ROS_INFO("Mirando a la izquierda");
    lookAt("base_footprint",0,5,0, pointheadclient);
    ROS_INFO("Mirando a la derecha");
    lookAt("base_footprint",0,-5,0, pointheadclient);
    ROS_INFO("Mirando al frente");
    lookAt("base_footprint",5,0,0, pointheadclient);
    delete pointheadclient;
    return EXIT_SUCCESS;
}