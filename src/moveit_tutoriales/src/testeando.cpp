#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
// Para mensajes de trajectoria de moveit
#include <moveit_msgs/DisplayTrajectory.h>
// Para Moveit
#include <moveit/planning_interface/planning_interface.h>
// Para MoveGroup
#include <moveit/move_group_interface/move_group.h>
// Para PlanningSceneInterface
#include <moveit/planning_scene_interface/planning_scene_interface.h>

/* Este programa utiliza modelos preincluidos en el paquete "pr2_moveit_config"
    - Inicializa brazo izquierdo
    - Define una pose (pose goal)
    - Planea trayectoria
    - La visualiza en RViz (previo launch de pr2_moveit_config/demo.launch)*/

int main(int argc, char **argv){
    // Inicializar nodo en ROS
    ros::init(argc, argv, "moveit_tutoriales");
    // Tutoriales tenían esto siempre en el main después de declarar el nodehandle.
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ROS_INFO("Comenzando ejecución");
    ros::NodeHandle nh;
    // Inicializar grupo a controlar
    ROS_INFO("Inicializado right_arm.");
    moveit::planning_interface::MoveGroup group("right_arm");
    // Para tratar con el mundo
    // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // Para ver los planes en Rviz
    ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path",1,true);
    moveit_msgs::DisplayTrajectory display_trajectory;
    ROS_INFO("Frame de referencia: %s", group.getPlanningFrame().c_str());
    ROS_INFO("End Effector link: %s", group.getEndEffectorLink().c_str());

    // Definir un pose goal
    geometry_msgs::Pose pose_goal;
    pose_goal.position.x = 0.5;
    pose_goal.position.y = -0.5;
    pose_goal.position.z = 1.1;
    pose_goal.orientation.w = 1;
    group.setPoseTarget(pose_goal);
    // Planear la trayectoria
    moveit::planning_interface::MoveGroup::Plan mi_plan;
    bool success = group.plan(mi_plan);
    ROS_INFO("Visualizando plan 1 (Pose Goal) %s",success?"":"FAILED");
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(5.0);
    if (success){
        ROS_INFO("Visualizando plan 1 nuevamente:");
        // Seteando campos del tópico para publicar
        display_trajectory.trajectory_start = mi_plan.start_state_;
        display_trajectory.trajectory.push_back(mi_plan.trajectory_);
        display_publisher.publish(display_trajectory);
        sleep(5.0); // Para alcanzar a ver la trayectoria
        ROS_INFO("Ejecutando plan");
        group.move();
    }
    ros::shutdown();
    return EXIT_SUCCESS;
}