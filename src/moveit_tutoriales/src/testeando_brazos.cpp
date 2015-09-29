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

// trajectory_msgs::JointTrajectory getNewHeadPose(){
//     trajectory_msgs::JointTrajectory new_pose;
//     new_pose.joint_names=['head_pan_joint', 'head_tilt_joint'];
//     new_pose.positions=(c_toggle ? [0,-1] : [0,1])
//     new_pose.velocities = [0,0];
//     new_pose.time_from_start.secs = 0;
//     new_pose.time_from_start.nsecs = 100000000;
//     c_toggle = !c_toggle;
//     return new_pose;
// }
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
void moveToPose(const geometry_msgs::Pose &_pose, moveit::planning_interface::MoveGroup &_group, moveit::planning_interface::MoveGroup::Plan &_planner)
{
    _group.setPoseTarget(_pose);
    ROS_INFO("Planning for pose (%.2f, %.2f, %.2f) - (%.2f, %.2f, %.2f, %.2f)", _pose.position.x, _pose.position.y, _pose.position.z, _pose.orientation.x, _pose.orientation.y, _pose.orientation.z, _pose.orientation.w);

    bool planDone = _group.plan(_planner);
    ROS_INFO("Pose planning %s", planDone ? "SUCCESSFUL" : "FAILED");

    if (planDone)
    {
        ROS_INFO("Executing plan");
        _group.move();
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
    // Ciclo eterno
    while (1){
    // Para cada grupo:
        // Setear pose_goal
        // geometry_msgs::Pose l_pose = getNewArmPose(brazo_izquierdo);
        //geometry_msgs::Pose r_pose = getNewArmPose(brazo_derecho);
/*        l_pose.position.x = (l_arm_toggle ? 0.5 : 0.5);
        l_pose.position.y = (l_arm_toggle ? 0.3 : 0.5);
        l_pose.position.z = 1.1;
        l_pose.orientation.w = 1;
        l_arm_toggle = !l_arm_toggle;
        r_pose.position.x = (r_arm_toggle ? 0.5 : 0.5);
        r_pose.position.y = (r_arm_toggle ? -0.3 : -0.5);
        r_pose.position.z = 1.1;
        r_pose.orientation.w = 1;
        r_arm_toggle = !r_arm_toggle;*/
        moveToPose(getNewArmPose(brazo_izquierdo.getName()), brazo_izquierdo, mi_plan);
        moveToPose(getNewArmPose(brazo_derecho.getName()), brazo_derecho, mi_plan);
        // brazo_derecho.setPoseTarget(getNewArmPose(brazo_derecho));
        // ROS_INFO("Nuevos pose_targets seteados");
        // // cabeza.setPoseTarget(getNewHeadPose());
        // // Planear
        // bool l_succ = brazo_izquierdo.plan(mi_plan);
        // // ROS_INFO("Planeé brazo izquierdo");
        // if (l_succ){
        // //     // Publicar plan
        // //     plan_brazo_derecho.trajectory_start = mi_plan.start_state_;
        // //     plan_brazo_derecho.trajectory.push_back(mi_plan.trajectory_);
        // //     plan_brazo_derecho
        //     ROS_INFO("Moviendo Brazo Izquierdo");
        //     brazo_izquierdo.move();
        // }
        // bool r_succ = brazo_derecho.plan(mi_plan);
        // if (r_succ){
        //     ROS_INFO("Moviendo Brazo Derecho");
        //     brazo_derecho.move();
        // }
        // c_succ = cabeza.plan(mi_plan);
        // if (l_succ){
        //     ROS_INFO("Moviendo Cabeza");
        //     cabeza.move();
        // }
    }
    ros::shutdown();
    return EXIT_SUCCESS;
}