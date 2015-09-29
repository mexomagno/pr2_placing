
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
// 
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <actionlib/client/simple_action_client.h>


/*
Para ejecutar esto es necesario:
    - Tener un pr2 corriendo
    - Haber ejecutado "rosrun pr2_move_base pr2_move_base.py"
Esto habilita los servidores de goals habilitados por el paquete pr2_move_base, que es el encargado de que todo esto funcione.

*/


// typedef para simplificar las cosas
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> BaseClient;

void walkTo(std::string frame_id, double x, double y, double z, BaseClient *bc){
    move_base_msgs::MoveBaseGoal goal;
    // Construir el punto al que la cabeza va a mirar
    // Se expresará respecto al frame "base_footprint"
    geometry_msgs::Pose pose;
    goal.target_pose.header.frame_id = frame_id;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;
    goal.target_pose.pose = pose;
    bc->sendGoal(goal);
    if (not bc->waitForResult(ros::Duration(15)))
        ROS_INFO("Timeout antes de llegar a la pose final");
}
int main(int argc,char** argv){
    ros::init(argc, argv, "movimiento_basico");
    
    // Cliente de movimiento
    BaseClient *base_client = new BaseClient("/pr2_move_base", true); // con true, no necesita ros::spin()
    // Esperar el server
    while (!base_client->waitForServer(ros::Duration(5.0))){
        ROS_INFO("Esperando al move_base server...");
    }
    ROS_INFO("Caminando a -1,0,0");
    walkTo("base_footprint",-1,0,0, base_client);
    delete base_client;
    return EXIT_SUCCESS;
}