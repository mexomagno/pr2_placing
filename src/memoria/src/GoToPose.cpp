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
const float PI = 3.1415;
const float DISTANCE_CORRECTION = 0.18;
const float DISTANCE_PRE_TURN = 0.15;
const float ANGLE_CORRECTION = 0.2;
// VARIABLES GLOBALES
ros::ServiceClient basedriver_client;
memoria::BaseDriver basedriver_srv;
// MÉTODOS
// Códigos de error
vector<string> errors(6);

int goToPose(geometry_msgs::PoseStamped& pose_goal){
    ROS_INFO("Obtenida pose ubicada en (%f,%f,%f) (frame: %s)",pose_goal.pose.position.x,pose_goal.pose.position.y,pose_goal.pose.position.z,pose_goal.header.frame_id.c_str());
    /*
        1) Mover base hasta un poco antes de llegar a punto final
        2) Girar la base a orientación requerida
        3) Mover un poco más (lo que falta)
     */
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
    ROS_INFO("Pose respecto al robot: (%f,%f,%f)",robot_pose.position.x,robot_pose.position.y,robot_pose.position.z);
/*    // Posición quedó rotada, contrarrestar
    double x = robot_pose.position.x;
    double y = robot_pose.position.y;
    robot_pose.position.x = x * cos(manual_angle2) - y * sin(manual_angle2);
    robot_pose.position.y = x * sin(manual_angle2) + y * cos(manual_angle2);
*/
    // 1) DESPLAZAR BASE
    if (robot_pose.position.x != 0 or robot_pose.position.y != 0){
        // Calcular distancia desde robot a nueva pose
        double distance = sqrt(robot_pose.position.x*robot_pose.position.x + robot_pose.position.y*robot_pose.position.y);
        // Calcular ángulo de pose actual a nueva pose
        //tf::Quaternion quat_angle(robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w);
        //double angle = quat_angle.getAngle();
        //ROS_INFO("Angulo nueva pose respecto a pose actual: %f",angle);
        // ***** Mover hacia pose
        ROS_INFO("Trasladando base");
        tf::Vector3 xend (robot_pose.position.x, robot_pose.position.y, robot_pose.position.z);
        tf::Vector3 anglestart (1,0,0);
        double angle = anglestart.angle(xend) * (robot_pose.position.y < 0 ? -1 : 1); // Resuelve problema de ángulo siempre positivo
        ROS_INFO("Angulo dirección desplazamiento: %f",angle*180.0/PI);
        // Corrección de distancia
        distance = distance - DISTANCE_CORRECTION - DISTANCE_PRE_TURN;
        basedriver_srv.request.distance = distance;
        basedriver_srv.request.angle = angle;
        if (not basedriver_client.call(basedriver_srv)){
            return 3;
        }
    }
    else{
        ROS_INFO("Recibida pose en misma posición");
    }
    // 2) ROTAR BASE
    ROS_INFO("Rotando base");
    // Rotar robot usando servicio BaseDriver
    tf::Quaternion quat_angle(robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w);
    double angle = quat_angle.getAngle();
    geometry_msgs::Quaternion q = robot_pose.orientation;
    double manual_angle = 2*acos(q.w);
    double manual_angle2 = atan2(2.0*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z));
    // Aplicar pequeña corrección al ángulo
    manual_angle2 = (manual_angle2 > 0 ? manual_angle2 - ANGLE_CORRECTION : manual_angle2 + ANGLE_CORRECTION);
    // Corrige angulos positivos
    ROS_INFO("Angulo con tf: %f, manual1: %f, manual2: %f", angle*180.0/PI, manual_angle*180.0/PI, manual_angle2*180.0/PI);
    angle = (angle > PI ? -(2*PI-angle) : angle);
    manual_angle = (manual_angle > PI ? -(2*PI-manual_angle) : manual_angle);
    ROS_INFO("Angulo tf corregido: %f, manual1 corregido: %f, manual2 corregido: %f", angle*180.0/PI, manual_angle*180.0/PI, manual_angle2*180.0/PI);
    basedriver_srv.request.distance = 0;
    basedriver_srv.request.angle = manual_angle2;
    if (not basedriver_client.call(basedriver_srv)){
        return 3;
    }
    // 3) MOVER ULTIMO POCO
    if (robot_pose.position.x != 0 or robot_pose.position.y != 0){
        ROS_INFO("Terminando de acercarse");
        basedriver_srv.request.distance = DISTANCE_PRE_TURN;
        basedriver_srv.request.angle = 0;
        if (not basedriver_client.call(basedriver_srv)){
            return 3;
        }
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
    if (retcode != 0){
        ROS_ERROR("Error: '%s'",errormsg.what.c_str());
    }
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