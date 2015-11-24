#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
// #include <trajectory_msgs/JointTrajectory.h>
// Para mensajes de trajectoria de moveit
#include <moveit_msgs/DisplayTrajectory.h>
// Para Moveit
#include <moveit/planning_interface/planning_interface.h>
// Para MoveGroup
#include <moveit/move_group_interface/move_group.h>
// Para PlanningSceneInterface
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <string>
#include <stdlib.h>
#include <csignal>
using namespace std;
/* Este programa se basa en testeando.cpp. Como tal, requiere haber lanzado demo.launch (del paquete de este mismo archivo)
    - Mueve partes del cuerpo en un loop eterno:
        -Brazo izquierdo
        -Brazo derecho
        -Cabeza    */

void signalHandler( int signum ){
    ros::shutdown();
    exit (EXIT_SUCCESS);
}
geometry_msgs::Pose newPose(float x, float y, float z, float ox, float oy, float oz, float w){
    geometry_msgs::Pose newpose;
    newpose.position.x  = x;
    newpose.position.y  = y;
    newpose.position.z  = z;
    newpose.orientation.x = ox;
    newpose.orientation.y = oy;
    newpose.orientation.z = oz;
    newpose.orientation.w  = w;
    return newpose;

}
void moveToPose(const geometry_msgs::PoseStamped &pose, moveit::planning_interface::MoveGroup &group, moveit::planning_interface::MoveGroup::Plan &planner){
    group.setPoseTarget(pose);
    ROS_INFO("Planeando para pose (%.2f, %.2f, %.2f) - (%.2f, %.2f, %.2f, %.2f)", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);

    bool planDone = group.plan(planner);
    ROS_INFO("Plan %s", planDone ? "EXITOSO" : "FALLIDO");

    if (planDone)
    {
        ROS_INFO("Ejecutando plan...");
        group.move();
    }
}
// void moveToPosition(const geometry_msgs::Point &position, moveit::planning_interface::MoveGroup &group, moveit::planning_interface::MoveGroup::Plan &planner){
//     group.setPositionTarget(position.x, position.y, position.z);
//     ROS_INFO("Planeando para posición (%.2f, %.2f, %.2f)", position.x, position.y, position.z);

//     bool planDone = group.plan(planner);
//     ROS_INFO("Plan %s", planDone ? "EXITOSO" : "FALLIDO");

//     if (planDone)
//     {
//         ROS_INFO("Ejecutando plan...");
//         group.move();
//     }
// }
// void moveToRandom(moveit::planning_interface::MoveGroup &group, moveit::planning_interface::MoveGroup::Plan &planner){
//     geometry_msgs::PoseStamped pose = group.getRandomPose();
//     group.setPoseTarget(pose);
//     bool planDone = group.plan(planner);
//     ROS_INFO("Plan %s", planDone ? "EXITOSO" : "FALLIDO");

//     if (planDone)
//     {
//         ROS_INFO("Ejecutando plan...");
//         group.move();
//     }
// }
int main(int argc, char **argv){
    // Recibir argumentos
    if (argc < 9){
        printf("Debe ingresar: [l|r] pose:[x, y, z] orientation:[x, y, z, w]\n");
        exit(1);
    }
    geometry_msgs::Pose pose = newPose(atof(argv[2]),atof(argv[3]),atof(argv[4]),atof(argv[5]),atof(argv[6]),atof(argv[7]),atof(argv[8]));
    geometry_msgs::PoseStamped final_pose;
    final_pose.pose = pose;
    final_pose.header.frame_id = "base_footprint";
    string brazo_str = (string(argv[1]).compare("l") == 0 ? "left_arm" : "right_arm");
    // Inicializar nodo en ROS
    ros::init(argc, argv, "moveit_tutoriales");
    // Tutoriales tenían esto siempre en el main después de declarar el nodehandle.
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ROS_INFO("Comenzando ejecución");
    ros::NodeHandle nh;
    // signal
    signal(SIGINT, signalHandler);
    // Inicializar grupos a controlar
    ROS_INFO("Inicializando '%s'", brazo_str.c_str());
    moveit::planning_interface::MoveGroup brazo(brazo_str.c_str());
    brazo.setPoseReferenceFrame("base_footprint");
    cout << "Current pose: " << brazo.getCurrentPose();
    ROS_INFO("Frame del plan: %s",brazo.getPlanningFrame().c_str());
    moveit::planning_interface::MoveGroup::Plan mi_plan;
    moveToPose(final_pose, brazo, mi_plan);
    ros::shutdown();
    return 0;
}