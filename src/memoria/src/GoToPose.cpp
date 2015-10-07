/* 
Servicio encargado de llevar inteligentemente al robot hacia cierta pose.
Recibe:   geometry_msgs/PoseStamped
Retorna:  memoria/ErrorMsg
*/

#include <string>
#include <vector>
#include <ros/ros.h>
#include <memoria/GoToPose.h>
#include <geometry_msgs/PoseStamped.h>
#include <memoria/ErrorMsg.h>

using namespace std;

// CONSTANTES
// Códigos de error
vector<string> errors(4);

int goToPose(geometry_msgs::PoseStamped& pose_goal){
    ROS_INFO("Obtenida pose ubicada en (%f,%f,%f) (frame: %s)",pose_goal.pose.position.x,pose_goal.pose.position.y,pose_goal.pose.position.z,pose_goal.header.frame_id.c_str());
    // Ver si se puede ir a esa pose
    // Mover hacia pose
    ROS_INFO("'Moviendo' hacia la pose");
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
    errors[3]="Error desconocido";
    
    ros::init(argc, argv, "go_to_pose_server");
    ros::NodeHandle nh;
    // Crear servicio
    ros::ServiceServer service = nh.advertiseService("go_to_pose",callback);
    ROS_INFO("Listo para recibir requests");
    ros::spin();
    return 0;
}