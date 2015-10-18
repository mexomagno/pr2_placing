/*
Servicio de movimiento de la base, de bajo nivel.
Es capaz de desplazar la base y de girarla, sin tener en cuenta los obstáculos del camino.

Recibe: float32 distancia.
        float32 angulo.  Si distancia es 0, se rota base en eje Z.
        string frame_id indicando 
Retorna: memoria/ErrorMsg


TODO:
    - Terminar ejecución si nodo que envía la orden muere en plena ejecución.
    - Retornar error cuando se recibe request en medio de una petición en curso
    - Implementar rebote para corrección al pasarse de posición o ángulo.
*/
#include <string>
#include <vector>
#include <ros/ros.h>
#include <csignal>
// Mensajes para otorgar servicio
//#include <memoria/BaseDriverMsg.h>
#include <memoria/BaseDriver.h>
#include <memoria/ErrorMsg.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>

using namespace std;
// CONSTANTES
ros::Publisher base_cmd_pub;
const string BASE_TOPIC = "/base_controller/command";
const string ODOM_TOPIC = "/base_odometry/odom";
const float WAIT_TF_TIMEOUT = 1.0;
const string ROBOT_FRAME = "/base_footprint";
const string WORLD_FRAME = "/odom_combined";
const float LOOP_FREQ = 10.0;
const float TWIST_VELOCITY = 1.5;
const float ANGULAR_VELOCITY = 3;
const double PI = 3.1415;
// VARIABLES GLOBALES
vector<string> errors(4);
bool store_odom = false;
tf::Vector3 last_position(0,0,0);
tf::Quaternion last_orientation(0,0,0,1);
// Variables auxiliares

// METODOS
void signalHandler( int signum ){
    ROS_INFO("Recibí CTRL+C. Terminando...");
    ros::shutdown();
    exit (EXIT_SUCCESS);
}
int turn(double angle){
    ROS_INFO("Girar en %f radianes",angle);
    ros::Rate rate(LOOP_FREQ);
    store_odom = true;
    rate.sleep();
    // Actualizar última posición
    ros::spinOnce();
    tf::Quaternion start_orientation = last_orientation;
    geometry_msgs::Twist base_cmd;
    base_cmd.angular.z = (angle<0?-1:1)*ANGULAR_VELOCITY;
    bool done = false;
    double real_angle = 0;
    double traveled_angle;
    while (!done and ros::ok()){
        ROS_INFO("Ciclo giro");
        base_cmd_pub.publish(base_cmd);
        rate.sleep();
        // Actualizar posición odométrica
        ros::spinOnce();
        traveled_angle = 2*start_orientation.angle(last_orientation);
        
        ROS_INFO("Angulo barrido: %f rad",traveled_angle);
        if (traveled_angle > (angle<0?-angle:angle)) done = true;
    }
    store_odom = false;
    if (done){
        ROS_INFO("Exito. Rotación real: %f rad", traveled_angle);
        return 0;
    }
    else{
        ROS_INFO("lol esto no debiera pasar.");
        return 3;
    }
}
int travel(double distance, double angle){
    ROS_INFO("Desplazar distancia=%f, angulo=%f",distance,angle);
    ros::Rate rate(LOOP_FREQ);
    store_odom = true;
    rate.sleep();
    ros::spinOnce();
    tf::Vector3 start_position = last_position;
    // Mensaje a enviar a la base
    geometry_msgs::Twist base_cmd;
    // ****** Calcular comando a enviar
    tf::Vector3 itongo((distance < 0 ? -1 : 1)*TWIST_VELOCITY,0,0),zaxis(0,0,1);
    // Rotar vector unitario
    itongo = itongo.rotate(zaxis, angle);
    ROS_INFO("Vector (%f,0,0) rotado en %f es (%f,%f,%f)",distance,angle,itongo.x(), itongo.y(), itongo.z());
    // Guardar en Twist
    base_cmd.linear.x = itongo.x();
    base_cmd.linear.y = itongo.y();
    base_cmd.linear.z = 0   ;//itongo.z()*TWIST_PONDERATOR;
    bool done = false;
    double real_distance = 0;
    double traveled_dist;
    while (!done and ros::ok()){
        ROS_INFO("Ciclo");
        // Enviar comando (ponderado)
        base_cmd_pub.publish(base_cmd);
        rate.sleep();
        // Actualizar posición odométrica
        ros::spinOnce();
        traveled_dist = start_position.distance(last_position);
        ROS_INFO("Distancia navegada = %f m",traveled_dist);
        if (traveled_dist > (distance<0 ? -distance:distance)) done = true;
    }
    store_odom = false;
    if (done){
        ROS_INFO("Exito. Desplazamiento real: %f m", traveled_dist);
        return 0;
    }
    else{
        ROS_INFO("lol esto no debiera pasar.");
        return 3;
    }
}
void odomCallback(const nav_msgs::Odometry::ConstPtr& odom){
    if (not store_odom)
        return;
    ROS_INFO("Recibida odometría: (%f,%f,%f) (%f,%f,%f,%f)",odom->pose.pose.position.x,odom->pose.pose.position.y,odom->pose.pose.position.z,odom->pose.pose.orientation.x,odom->pose.pose.orientation.y,odom->pose.pose.orientation.z,odom->pose.pose.orientation.w);
    // Actualizar última posición
    last_position.setX(odom->pose.pose.position.x);
    last_position.setY(odom->pose.pose.position.y);
    last_position.setZ(odom->pose.pose.position.z);

    // Actualizar última orientación
    last_orientation.setX(odom->pose.pose.orientation.x);
    last_orientation.setY(odom->pose.pose.orientation.y);
    last_orientation.setZ(odom->pose.pose.orientation.z);
    last_orientation.setW(odom->pose.pose.orientation.w);
}

bool callback(memoria::BaseDriver::Request &request, memoria::BaseDriver::Response& response){
    ROS_INFO("Recibido request: distance=%f, angle=%f",request.distance, request.angle);
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
    ros::NodeHandle nh;
    signal(SIGINT, signalHandler);
    // Subscripción a mensajes de odometría
    // ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(ODOM_TOPIC,1,odomCallback);
    ros::Subscriber odom_sub = nh.subscribe(ODOM_TOPIC,1,odomCallback);
    // Publicador mensajes para base cmd
    base_cmd_pub = nh.advertise<geometry_msgs::Twist>(BASE_TOPIC,1);
    // Crear servicio
    ros::ServiceServer service = nh.advertiseService("base_driver", callback);
    ROS_INFO("Listo para recibir requests en base_driver");
    ros::spin();
    return 0;
}
