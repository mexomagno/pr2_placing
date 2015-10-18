/* 
Servicio encargado de llevar inteligentemente al robot hacia cierta pose.
Recibe:   geometry_msgs/PoseStamped
Retorna:  memoria/ErrorMsg











TODO:
    - Implementar navegación real


Particularidades pr2 navigation:
    - Exportar ROBOT=sim cuando sea simulador y pr2 cuando no.
    // - roslaunch pr2_2dnav_local pr2_2dnav.launch

Problemas:
    - El módulo pr2_navigation_self_filter no está soportado para hydro 
        - http://wiki.ros.org/pr2_navigation_self_filter
        - https://groups.google.com/forum/#!topic/moveit-users/5UQ7cQW531w


*/


#include <string>
#include <vector>
#include <math.h>
#include <ros/ros.h>
#include <memoria/GoToPose.h>
#include <memoria/BaseDriver.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <memoria/ErrorMsg.h>
#include <tf/transform_listener.h>

using namespace std;

// CONSTANTES
const string ROBOT_FRAME = "base_footprint";
const float WAIT_TF_TIMEOUT = 1.0;
// VARIABLES GLOBALES
ros::ServiceClient basedriver_client;
memoria::BaseDriver basedriver_srv;
// MÉTODOS
// Códigos de error
vector<string> errors(6);

int goToPose(geometry_msgs::PoseStamped& pose_goal){
    ROS_INFO("Obtenida pose ubicada en (%f,%f,%f) (frame: %s)",pose_goal.pose.position.x,pose_goal.pose.position.y,pose_goal.pose.position.z,pose_goal.header.frame_id.c_str());
    // ***** Ver si se puede ir a esa pose
    // ***** Calcular distancia y ángulo a enviar al BaseDriver
    /* Este punto es sensible. Depende del frame_id de la pose.
    El servicio BaseDriver recibe una distancia medida desde la base, y un ángulo medido en eje Z de la base, con 0 en eje X.
    Por esto, acá hay que transformar la pose desde el frame que venga, hasta la base del robot.
    */
    geometry_msgs::Pose robot_pose;
    // Transformar pose recibida al frame del robot
    if (ROBOT_FRAME.compare(pose_goal.header.frame_id) != 0){
        tf::TransformListener tf_listener;
        tf::StampedTransform stamped_tf;
        ros::Time tf_time = ros::Time(0);
        try{
            ROS_INFO("Esperando transformación disponible...");
            if (not tf_listener.waitForTransform(pose_goal.header.frame_id, ROBOT_FRAME, tf_time, ros::Duration(WAIT_TF_TIMEOUT))){
                ROS_ERROR("Transformación no pudo ser obtenida en %f segundos",WAIT_TF_TIMEOUT);
                // Salir del callback
                return 4;
            }
            tf_listener.lookupTransform(pose_goal.header.frame_id, ROBOT_FRAME, ros::Time(0), stamped_tf);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("Excepción al obtener transformación: %s",ex.what());
            return 4;
        }
        geometry_msgs::PoseStamped robot_pose_stamped;
        tf_listener.transformPose(ROBOT_FRAME, pose_goal, robot_pose_stamped);
        robot_pose = robot_pose_stamped.pose;
    }
    else{
        robot_pose = pose_goal.pose;
    }
    ROS_INFO("Pose Robot: (%f,%f,%f)",robot_pose.position.x,robot_pose.position.y,robot_pose.position.z);
    // Calcular distancia
    double distance = sqrt(robot_pose.position.x*robot_pose.position.x + robot_pose.position.y*robot_pose.position.y);
    // Calcular ángulo (a partir de quaternion)
    tf::Quaternion quat_angle(robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w);
    double angle = quat_angle.getAngle();
    ROS_INFO("Angulo Pose: %f",angle);
    // ***** Mover hacia pose
    ROS_INFO("1) Rotando base");
    // Rotar robot usando servicio BaseDriver
    basedriver_srv.request.distance = 0;
    basedriver_srv.request.angle = angle;
    if (not basedriver_client.call(basedriver_srv)){
        return 3;
    }
    ROS_INFO("2) Trasladando base");
    // Desplazar robot usando servicio BaseDriver
    basedriver_srv.request.distance = distance;
    basedriver_srv.request.angle = 0;
    if (not basedriver_client.call(basedriver_srv)){
        return 3;
    }
    ROS_INFO("Se mueve con éxito");
    return 0;
}
bool callback(memoria::GoToPose::Request& request, memoria::GoToPose::Response& response){
    ROS_INFO("Request entrante");
    // Ir a la pose
    int retcode = goToPose(request.pose);
    // Crear código de error
    memoria::ErrorMsg errormsg;
    errormsg.retcode = retcode;
    errormsg.what = errors[errormsg.retcode];
    response.error = errormsg;
    return true;
}
int main(int argc, char **argv){
    errors[0]="";
    errors[1]="Posegoal inalcanzable";
    errors[2]="Imposible conectar con servidor";
    errors[3]="Error al llamar al servicio 'base_driver'";
    errors[4]="Imposible obtener Transform";
    errors[5]="Error desconocido";
    
    ros::init(argc, argv, "go_to_pose_server");
    ros::NodeHandle nh;
    // "Subscribirse" al servicio 'base_driver'
    basedriver_client = nh.serviceClient<memoria::BaseDriver>("base_driver");
    // Crear servicio
    ros::ServiceServer service = nh.advertiseService("go_to_pose",callback);
    ROS_INFO("Listo para recibir requests");
    ros::spin();
    return 0;
}