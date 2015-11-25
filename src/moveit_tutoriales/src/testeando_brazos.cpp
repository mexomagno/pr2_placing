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
bool r_arm_toggle,l_arm_toggle,c_toggle;

void signalHandler( int signum ){
    ros::shutdown();
    exit (EXIT_SUCCESS);
}

geometry_msgs::Pose getNewArmPose(const string &name){
    if (name.compare("left_arm") != 0 and name.compare("right_arm") != 0){
        ROS_INFO("ERROR: getName retornó algo raro: %s", name.c_str());
        ros::shutdown();
        exit(EXIT_FAILURE);
    }
    ROS_INFO("Seteando nueva pose para %s",name.c_str());
    geometry_msgs::Pose pose;
    // Retornar nueva pose según valor del toggle
    switch (name.compare("right_arm")){
        // brazo derecho
        case 0:
            pose.position.x = (r_arm_toggle ? 0.5 : 0.5 );
            pose.position.y = (r_arm_toggle ? -0.3 : -0.5 );
            pose.position.z = (r_arm_toggle ? 1 : 1 );
            pose.orientation.w = (r_arm_toggle ? 1 : 1 );
            r_arm_toggle = !r_arm_toggle;
            break;
        // brazo izquierdo
        default:
            l_arm_toggle = !l_arm_toggle;
            pose.position.x = (l_arm_toggle ? 0.5 : 0.5 );
            pose.position.y = (l_arm_toggle ? 0.3 : 0.5 );
            pose.position.z = (l_arm_toggle ? 1 : 1 );
            pose.orientation.w = (l_arm_toggle ? 1 : 1 );
    }
    ROS_INFO("Seteada nueva pose");
    return pose;
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
void moveToPose(const geometry_msgs::Pose &pose, moveit::planning_interface::MoveGroup &group, moveit::planning_interface::MoveGroup::Plan &planner){
    group.setPoseTarget(pose);
    ROS_INFO("Planeando para pose (%.2f, %.2f, %.2f) - (%.2f, %.2f, %.2f, %.2f)", pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

    bool planDone = group.plan(planner);
    ROS_INFO("Plan %s", planDone ? "EXITOSO" : "FALLIDO");

    if (planDone)
    {
        ROS_INFO("Ejecutando plan...");
        group.move();
    }
}
void moveToPosition(const geometry_msgs::Point &position, moveit::planning_interface::MoveGroup &group, moveit::planning_interface::MoveGroup::Plan &planner){
    group.setPositionTarget(position.x, position.y, position.z);
    ROS_INFO("Planeando para posición (%.2f, %.2f, %.2f)", position.x, position.y, position.z);

    bool planDone = group.plan(planner);
    ROS_INFO("Plan %s", planDone ? "EXITOSO" : "FALLIDO");

    if (planDone)
    {
        ROS_INFO("Ejecutando plan...");
        group.move();
    }
}
void moveToRandom(moveit::planning_interface::MoveGroup &group, moveit::planning_interface::MoveGroup::Plan &planner){
    geometry_msgs::PoseStamped pose = group.getRandomPose();
    group.setPoseTarget(pose);
    bool planDone = group.plan(planner);
    ROS_INFO("Plan %s", planDone ? "EXITOSO" : "FALLIDO");

    if (planDone)
    {
        ROS_INFO("Ejecutando plan...");
        group.move();
    }
}
int main(int argc, char **argv){
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
    ROS_INFO("Inicializando Brazo Izquierdo");
    moveit::planning_interface::MoveGroup brazo_izquierdo("left_arm");
    ROS_INFO("Inicializando Brazo Derecho");
    moveit::planning_interface::MoveGroup brazo_derecho("right_arm");
    // ROS_INFO("Inicializando Cabeza");
    // moveit::planning_interface::MoveGroup cabeza("head");
    r_arm_toggle = l_arm_toggle = c_toggle = false;
    // Publica tópico con motionplanning
    // ROS_INFO("Publicando tópicos");
    // ros::Publisher plan_brazo_derecho = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/plan_brazo_derecho",1,true);
    // ros::Publisher plan_brazo_izquierdo = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/plan_brazo_izquierdo",1,true);
    // ros::Publisher plan_cabeza = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/plan_cabeza",1,true);
    ROS_INFO("Brazo izquierdo: frame del plan: %s, end effector: %s",brazo_izquierdo.getPlanningFrame().c_str(), brazo_izquierdo.getEndEffectorLink().c_str());
    ROS_INFO("Brazo derecho: frame del plan: %s, end effector: %s",brazo_derecho.getPlanningFrame().c_str(), brazo_derecho.getEndEffectorLink().c_str());
    // ROS_INFO("Cabeza: frame del plan: %s, end effector: %s",cabeza.getPlanningFrame().c_str(), cabeza.getEndEffectorLink().c_str());
    // bool l_succ= false , r_succ = false, c_succ= false;
    moveit::planning_interface::MoveGroup::Plan mi_plan;
    // Defino par de poses para cada brazo
    vector<geometry_msgs::Pose> poses_iz, poses_der;
    poses_iz.push_back(newPose(0.5,0.3,1,0,0,0,1));
    poses_iz.push_back(newPose(0.5,0.5,1,0,0,0,1));
    poses_der.push_back(newPose(0.5,-0.3,1,0,0,0,1));
    poses_der.push_back(newPose(0.5,-0.5,1,0,0,0,1));
    char toggle = 0;
    // Ciclo eterno
    while (1){
        // moveToPosition(poses_iz[toggle].position, brazo_izquierdo, mi_plan);
        // moveToPosition(poses_der[toggle].position, brazo_derecho, mi_plan);
        // moveToPose(poses_iz[toggle], brazo_izquierdo, mi_plan);
        // moveToPose(poses_der[toggle], brazo_derecho, mi_plan);
        // toggle = (toggle + 1)%2;
        moveToRandom(brazo_izquierdo, mi_plan);
        moveToRandom(brazo_derecho, mi_plan);
    }
    ros::shutdown();
    return EXIT_SUCCESS;
}