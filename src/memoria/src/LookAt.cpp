/*
Este nodo se preocupa de los movimientos de cabeza del PR2
Está implementado como un servicio de ROS.
Recibe:
    - Mensaje personalizado de ROS memoria/LookAtMsg, con:
        string frame_id = Frame de las coordenadas siguientes
        geometry_msgs/Vector3 vector = coordenadas, si rotate = 0, o (Yaw ,Pitch,NADA) si rotate = 1
        int32 rotate = 0 o 1.
Retorna:
    - 0 si todo ok
    - 1 si hubo error
    - 2 si se produjo timeout


Mejoras:
    - Reparar precisión del punto especificado
    - Reparar "waitForResult" que retorna antes de terminar movimiento
    - Reparar rotación en Pitch

*/


#include <ros/ros.h>
// Mensajes para Action Interface de la cabeza
#include <pr2_controllers_msgs/PointHeadGoal.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
// Mensaje para otorgar servicio
#include <memoria/LookAt.h>
#include <memoria/LookAtMsg.h>
//#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <string>
#include <math.h>

using namespace std;
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

// CONSTANTES
const string LOOKAT_TOPIC = "memoria/lookat";
const string ROBOT_FRAME = "/base_footprint";
const string CAMERA_FRAME = "high_def_frame";
const string KINECT_FRAME = "head_mount_kinect_ir_optical_frame";
const string POINT_HEAD_CONTROLLER = "/head_traj_controller/point_head_action";
const float HEAD_MAX_VELOCITY = 0.8;
const float HEAD_MIN_DURATION = 0.1;
const float HEAD_TIMEOUT = 15;
const bool RELATIVE_ROTATION = false; // if true, rotación se mide desde última pose. Else, desde pose base (mirando al frente)
// typedef para simplificar las cosas

PointHeadClient *phc;
geometry_msgs::Point lastpoint;

int lookAt(string frame_id, double x, double y, double z){
    geometry_msgs::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    pr2_controllers_msgs::PointHeadGoal goal;
    // Construir el punto al que la cabeza va a mirar
    // Se expresará respecto al frame "base_footprint"
    //geometry_msgs::Point punto;
    goal.target.header.frame_id = frame_id;
    goal.target.point = point;
    // Se quiere que el eje X de la cámara apunte al punto
    goal.pointing_frame = KINECT_FRAME; // podría ser otro si se quisiera.
    goal.pointing_axis.x = 0;
    goal.pointing_axis.y = 0;
    goal.pointing_axis.z = 1;
    // Limitar velocidades
    goal.max_velocity = HEAD_MAX_VELOCITY; // rad/s
    goal.min_duration = ros::Duration(HEAD_MIN_DURATION);
    phc->sendGoal(goal);
    if (not phc->waitForResult(ros::Duration(HEAD_TIMEOUT))){
        ROS_INFO("Timeout antes de llegar a la pose final");
        return 2;
    }
    lastpoint = point;
    return 0;
}

bool callback(memoria::LookAt::Request& lookatmsg, memoria::LookAt::Response& ret){
    int retcode = 0;
    // Validar valores
    double x, y, z;
    if (lookatmsg.lookatmsg.rotate != 0){
        // Calcular punto resultante de rotación
        double yaw = lookatmsg.lookatmsg.vector.x;
        double pitch = lookatmsg.lookatmsg.vector.y;
        ROS_INFO("Se pidio rotar yaw = %f, pitch = %f",yaw,pitch);
        // Rotar respecto a Z
        /*
        Rz = | cosW -sinW 0 |
             | sinW cosW  0 |
             |   0    0   1 |
        */
        if (RELATIVE_ROTATION){
            x = lastpoint.x*cos(yaw)+lastpoint.y*-sin(yaw);
            y = lastpoint.x*sin(yaw)+lastpoint.y*cos(yaw);
        }
        else{
            // Vector base es: X=módulo de x y, Y=Z=0
            double modulo = sqrt(lastpoint.x*lastpoint.x + lastpoint.y*lastpoint.y);
            x = modulo*cos(yaw);
            y = modulo*sin(yaw);
        }
        z = lastpoint.z;
        // Rotar respecto a Y
        /*
        Ry = | cosW  0 sinW |
             |   0   1   0  |
             | -sinW 0 cosW |
        */
        x = x*cos(pitch) + z*sin(pitch);
        z = x*-sin(pitch) + z*cos(pitch);
    }
    else {
        x = lookatmsg.lookatmsg.vector.x;
        y = lookatmsg.lookatmsg.vector.y;
        z = lookatmsg.lookatmsg.vector.z;
    }
    // Mirar hacia dirección indicada
    ROS_INFO("Mirando al punto %f,%f,%f...\n",x,y,z);
    retcode = lookAt(lookatmsg.lookatmsg.frame_id,x,y,z);
    ret.error = retcode;
}

int main(int argc,char** argv){
    ros::init(argc, argv, "lookat_server");
    ros::NodeHandle nh;
    // Cliente de movimiento
    phc = new PointHeadClient(POINT_HEAD_CONTROLLER, true);
    // Esperar el server
    while (!phc->waitForServer(ros::Duration(5.0))){
        ROS_INFO("Esperando al point_head_action server...");
    }
    // Setear punto inicial de mira
    ROS_INFO("Mirando hacia el frente...");
    lookAt(ROBOT_FRAME,10,0,1.5); // Aproximadamente mirar al frente
    // Crear servicio
    ros::ServiceServer service = nh.advertiseService("look_at", callback);
    ROS_INFO("Listo para recibir requests en look_at");
    // Suscriptor peticiones lookat
    ros::spin();
    delete phc;
    return 0;
}