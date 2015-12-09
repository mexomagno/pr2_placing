/*
Información sobre colisiones del octree
https://groups.google.com/forum/#!topic/moveit-users/EI73skgnGVk

*/
#include <string>
#include <vector>
#include <csignal>
#include <ros/ros.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf/transform_listener.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <gazebo_msgs/GetModelState.h>
#include <memoria/GripperDriver.h>
using namespace std;
typedef planning_scene_monitor::PlanningSceneMonitorPtr PlanScenMon;
// CONSTANTES
const float lata_height=0.07;
const float lata_width=0.07;
const float lata_length=0.15;


// VARIABLES GLOBALES

//PlanScenMon psm;
//tf::TransformListener tfl;
ros::Publisher planning_scene_pub;

// Clientes
ros::ServiceClient getmodelstate_client;
gazebo_msgs::GetModelState gms_srv;
ros::ServiceClient gripperdriver_client;
memoria::GripperDriver gripperdriver_srv;
ros::ServiceClient getplanningscene_client;

// auxiliares
ros::Publisher point_pub;
ros::Publisher point_pub2;


void signalHandler( int signum ){
    exit(0);
}
bool moveToPose(const geometry_msgs::Pose &_pose, moveit::planning_interface::MoveGroup &_group, moveit::planning_interface::MoveGroup::Plan &_planner)
{
    _group.setPoseTarget(_pose);
    printf("pose reference frame: %s\n", _group.getPoseReferenceFrame().c_str());
    _group.setPoseReferenceFrame("/odom_combined");
    //_group.setGoalOrientationTolerance(0.01);// Unidades??
    //_group.setGoalPositionTolerance(0.01); // ???
    _group.setGoalTolerance(0.01); // Radio de esfera de incertidumbre;
    _group.setNumPlanningAttempts(20); // Cantidad de intentos de plan. Default: 1
    _group.setPlanningTime(5); // Tiempo máximo que se puede tomar para planear. Default, 5s parece.

    ROS_INFO("Planning for pose (%.2f, %.2f, %.2f) - (%.2f, %.2f, %.2f, %.2f)", _pose.position.x, _pose.position.y, _pose.position.z, _pose.orientation.x, _pose.orientation.y, _pose.orientation.z, _pose.orientation.w);

    bool planDone = _group.plan(_planner);
    ROS_INFO("Planning %s", planDone ? "EXITOSO" : "FALLA");

    if (planDone)
    {
        ROS_INFO("Executing plan");
        _group.move();
    }
    return planDone;
}
/*void publishBox(geometry_msgs::PoseStamped object_pose){
    // Publicar
    moveit_msgs::CollisionObject object;
    object.header = object_pose.header;
    object.id = "transport_box";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = lata_height*2.0;
    primitive.dimensions[1] = lata_width;
    primitive.dimensions[2] = lata_length;

    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(object_pose.pose);
    //object.operation = attached_object.object.ADD;
    moveit_msgs::PlanningScene planning_scene_add;
    planning_scene_add.world.collision_objects.push_back(object);
    planning_scene_add.is_diff = true;
    planning_scene_pub.publish(planning_scene_add);
    moveit_msgs::GetPlanningScene srv;
    srv.request.components.components = moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES;
    bool object_in_world = false;
    while(!object_in_world){
        ROS_ERROR("Esperando que aparezca la caja");
        if (getplanningscene_client.call(srv)){
            for (int i=0; i<(int)srv.response.scene.world.collision_object.size(); ++i){
                if (srv.response.scene.world.collision_objects[i].id == "transport_box")
                    object_in_world = true;
            }
        }
    }
}*/
void pick(string object, moveit::planning_interface::MoveGroup &group){
    ROS_INFO("COMIENZA PICK");

    /* Esta función es un auxiliar para poder hacer grasp en gazebo*/
    // ###### Obtener posición del objeto
    ROS_INFO("Obteniendo posición de '%s'",object.c_str());
    // Pedir pose del objeto
    gms_srv.request.model_name = object;
    if (not getmodelstate_client.call(gms_srv)){
        ROS_ERROR("Error al llamar servicio de Gazebo 'get_model_state'");
        return;
    }
    geometry_msgs::Pose pose_lata, pose_detras_lata;
    pose_lata = gms_srv.response.pose;
    pose_detras_lata = gms_srv.response.pose;
    pose_detras_lata.position.x -= 0.2;
    pose_detras_lata.orientation.x = pose_detras_lata.orientation.y = pose_detras_lata.orientation.z = pose_lata.orientation.x = pose_lata.orientation.y = pose_lata.orientation.z = 0;
    ROS_INFO("'%s' en (%f, %f, %f)\nPublicando",object.c_str(),pose_lata.position.x, pose_lata.position.y, pose_lata.position.z);
    geometry_msgs::PointStamped object_position;
    object_position.header.frame_id = "odom_combined";
    object_position.point = pose_lata.position;
    point_pub.publish(object_position);
    pose_detras_lata.orientation.w = pose_lata.orientation.w = 1;
    // ###### Situarse encima del objeto
    ROS_INFO("Situándose detras de la lata");
    moveit::planning_interface::MoveGroup::Plan planner;
    if (not moveToPose(pose_detras_lata, group, planner)){
        ROS_ERROR("Error al mover grupo");
        return;
    }
    geometry_msgs::PointStamped gripper_position;
    gripper_position.header.frame_id = "odom_combined";
    gripper_position.point = pose_detras_lata.position;
    point_pub2.publish(gripper_position);
    // ###### Abrir gripper al máximo
    ROS_INFO("Abriendo Gripper");
    bool isright = (group.getName().compare("right_arm") == 0? true : false);
    gripperdriver_srv.request.right = isright;
    gripperdriver_srv.request.opening = 1;
    if (not gripperdriver_client.call(gripperdriver_srv)){
        ROS_ERROR("Error al llamar servicio 'gripper_driver'");
        ROS_ERROR("Error retornado: '%s'",gripperdriver_srv.response.error.what.c_str());
        return;
    }

    // ###### Bajar a pose del objeto
    ROS_INFO("Envolviendo lata con gripper");
    if (not moveToPose(pose_lata,group,planner)){
        ROS_ERROR("Error al mover grupo");
        return;
    }
    // ###### Cerrar gripper hasta máximo esfuerzo
    ROS_INFO("Cerrando Gripper");
    gripperdriver_srv.request.right = isright;
    gripperdriver_srv.request.opening = 0.1;
    gripperdriver_srv.request.max_effort = 100;
    if (not gripperdriver_client.call(gripperdriver_srv)){
        ROS_ERROR("Error al llamar servicio 'gripper_driver'");
        ROS_ERROR("Error retornado: '%s'",gripperdriver_srv.response.error.what.c_str());
        return;
    }
    // ###### Levantar gripper a pose encima del objeto
    ROS_INFO("Subiendo objeto");
    if (not moveToPose(pose_detras_lata, group, planner)){
        ROS_ERROR("Error al mover grupo");
        return;
    }
    ROS_INFO("TERMINA PICK");

}
void prePick(string object, moveit::planning_interface::MoveGroup &group){
    ROS_INFO("COMIENZA PICK");

    /* Esta función es un auxiliar para poder hacer grasp en gazebo*/
    // ###### Obtener posición del objeto
    ROS_INFO("Obteniendo posición de '%s'",object.c_str());
    // Pedir pose del objeto
    gms_srv.request.model_name = object;
    if (not getmodelstate_client.call(gms_srv)){
        ROS_ERROR("Error al llamar servicio de Gazebo 'get_model_state'");
        return;
    }
    geometry_msgs::Pose pose_lata = gms_srv.response.pose;
    pose_lata.position.x -=0.2;
    pose_lata.position.z +=0.03;
    pose_lata.orientation.x = pose_lata.orientation.y = pose_lata.orientation.z = 0;
    pose_lata.orientation.w = 1;
    // Guardar la pose actual
    //geometry_msgs::PoseStamped prev_pose = group.getCurrentPose();
    ROS_INFO("Situándose detras de la lata");
    moveit::planning_interface::MoveGroup::Plan planner;
    if (not moveToPose(pose_lata, group, planner)){
        ROS_ERROR("Error al mover grupo");
        return;
    }
    ROS_INFO("Abriendo Gripper");
    bool isright = (group.getName().compare("right_arm") == 0? true : false);
    gripperdriver_srv.request.right = isright;
    gripperdriver_srv.request.opening = 1;
    if (not gripperdriver_client.call(gripperdriver_srv)){
        ROS_ERROR("Error al llamar servicio 'gripper_driver'");
        ROS_ERROR("Error retornado: '%s'",gripperdriver_srv.response.error.what.c_str());
        return;
    }
    ROS_INFO("Esperando que posicione objeto en el gripper...");
    ros::Duration(1.0).sleep();    
    // ###### Cerrar gripper hasta máximo esfuerzo
    ROS_INFO("Cerrando Gripper");
    gripperdriver_srv.request.right = isright;
    gripperdriver_srv.request.opening = 0.1;
    gripperdriver_srv.request.max_effort = 10;
    if (not gripperdriver_client.call(gripperdriver_srv)){
        ROS_ERROR("Error al llamar servicio 'gripper_driver'");
        ROS_ERROR("Error retornado: '%s'",gripperdriver_srv.response.error.what.c_str());
        return;
    }
    // ###### Levantar gripper a pose encima del objeto
    ROS_INFO("Moviendo objeto a posición de observación");
    group.setPoseReferenceFrame("base_footprint");
    geometry_msgs::Pose final_pose;
    final_pose.position.x = 1.3;
    final_pose.position.y = 0;
    final_pose.position.z = 1.1;
    final_pose.orientation.x = 0;
    final_pose.orientation.y = 0;
    final_pose.orientation.z = 0;
    final_pose.orientation.w = 1;

    if (not moveToPose(final_pose, group, planner)){
        ROS_ERROR("Error al mover grupo");
        return;
    }
    ROS_INFO("TERMINA PICK");
}
int main(int argc, char **argv){
    if (argc < 2 ){
        printf("ERROR: Debe ingresar brazo a usar [l|r]\n");
        exit(1);
    }
    if (argc < 3){
        printf("ERROR: Debe ingresar nombre de objeto a graspiar zi\n");
        exit(1);
    }
    char brazo = argv[1][0];
    string grasp_object = argv[2];
    ros::init(argc,argv,"grasping_test");
    ros::NodeHandle nh;
    point_pub = nh.advertise<geometry_msgs::PointStamped>("posicion_objeto",1);
    point_pub2 = nh.advertise<geometry_msgs::PointStamped>("posicion_gripper",1);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // publicador de planning_scene
    //planning_scene_pub = nh.advertise<moveit_msgs::PlanningScene>("planning_scene_2",1);
    //getplanningscene_client = nh.serviceClient<moveit_msgs::PlanningScene>("/moveit_msgs/GetPlanningScene");
    // Crear planning_scene_monitor
    //psm = PlanScenMon(new PlanScenMon("robot_description", tfl, "planning_scene_monitor"));

    getmodelstate_client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    gripperdriver_client = nh.serviceClient<memoria::GripperDriver>("gripper_driver");
    ROS_INFO("Iniciado");
    signal(SIGINT, signalHandler);
    // crear grupo brazo derecho
    moveit::planning_interface::MoveGroup r_arm((brazo == 'l' ? "left_arm" : "right_arm"));
    moveit::planning_interface::MoveGroup::Plan plan;
    // Tomar lata de cocacola
    ROS_INFO("Comenzando pick de '%s'",grasp_object.c_str());
    prePick(grasp_object, r_arm);
    return 0;
}
