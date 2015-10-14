/*
Servicio de movimiento de la base, de bajo nivel.
Es capaz de desplazar la base y de girarla, sin tener en cuenta los obstáculos del camino.

Recibe: float32 distancia.
        float32 angulo.  Si distancia es 0, se rota base en eje Z.
        string frame_id indicando 
Retorna: memoria/ErrorMsg

*/
#include <string>
#include <vector>
#include <ros/ros.h>
// Mensajes para otorgar servicio
//#include <memoria/BaseDriverMsg.h>
#include <memoria/BaseDriver.h>
#include <memoria/ErrorMsg.h>

#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>

using namespace std;
// CONSTANTES
ros::Publisher base_cmd_pub;
const string BASE_TOPIC = "cmd_vel";
const float WAIT_TF_TIMEOUT = 1.0;
const string ROBOT_FRAME = "/base_footprint";
const string WORLD_FRAME = "/odom_combined";
const float LOOP_FREQ = 10.0;
const float TWIST_PONDERATOR = 0.25;
const double PI = 3.1415;
// VARIABLES GLOBALES
ros::NodeHandle nh;
vector<string> errors(4);
// Variables auxiliares

// METODOS

int turn(double angle){
    return 0;
}
int travel(double distance, double angle){
    tf::TransformListener tf_listener;
    tf::StampedTransform start_tf, current_tf;
    if (not tf_listener.waitForTransform(ROBOT_FRAME, WORLD_FRAME, ros::Time(0), ros::Duration(WAIT_TF_TIMEOUT))){
        ROS_ERROR("Transformación no pudo ser obtenida en %f s",WAIT_TF_TIMEOUT);
        return 2;
    }
    // Mensaje a enviar a la base
    geometry_msgs::Twist base_cmd;
    // ****** Calcular comando a enviar
    tf::Vector3 itongo(1,0,0),zaxis(0,0,1);
    // Rotar vector unitario
    itongo.rotate(zaxis, angle);
    ROS_INFO("Vector (1,0,0) rotado en %f es (%f,%f,%f)",angle,itongo.x(), itongo.y(), itongo.z());
    // Guardar en Twist
    base_cmd.linear.x = itongo.x()*TWIST_PONDERATOR;
    base_cmd.linear.y = itongo.y()*TWIST_PONDERATOR;
    base_cmd.linear.z = 0   ;//itongo.z()*TWIST_PONDERATOR;
    ros::Rate rate(LOOP_FREQ);
    bool done = false;
    double real_distance = 0;
    while (!done and nh.ok()){
        // Enviar comando (ponderado)
        base_cmd_pub.publish(base_cmd);
        rate.sleep();
        // Obtener transformación actual
        try{
            tf_listener.lookupTransform(ROBOT_FRAME, WORLD_FRAME, ros::Time(0), current_tf);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("Excepción al obtener transformación: %s", ex.what());
            return 2;
        }
        // Checkear distancia viajada
        tf::Transform relative_tf = start_tf.inverse() * current_tf;
        double traveled_dist = relative_tf.getOrigin().length();
        real_distance += traveled_dist;
        if (traveled_dist > distance) done = true;
    }
    if (done){
        ROS_INFO("Exito. Desplazamiento real: %fm", real_distance);
        return 0;
    }
    else{
        ROS_INFO("lol esto no debiera pasar.");
        return 3;
    }
}

bool callback(memoria::BaseDriver::Request &request, memoria::BaseDriver::Response& response){
    int retcode = 0;
    double distance = request.distance;
    double angle = request.angle;
    // Validar inputs
    if (angle < -2*PI or angle > 2*PI){
        retcode = 0;
    }
    // Mover
    else{
        if (distance == 0)
            retcode = turn(angle);
        else
            retcode = travel(distance,angle);
    }
    // Retornar error
    memoria::ErrorMsg errormsg;
    errormsg.retcode = retcode;
    errormsg.what = errors[errormsg.retcode];
    response.error = errormsg;
    return true;
}

int main(int argc, char **argv){
    errors[0]="";
    errors[1]="Inputs inválidos";
    errors[2]="Error al obtener transform";
    errors[3]="Error desconocido";
    ros::init(argc, argv, "base_driver");
    // Publicador mensajes para base cmd
    base_cmd_pub = nh.advertise<geometry_msgs::Twist>(BASE_TOPIC,1);
    // Crear servicio
    ros::ServiceServer service = nh.advertiseService("base_driver", callback);
    ROS_INFO("Listo para recibir requests en base_driver");
    ros::spin();
    return 0;
}
