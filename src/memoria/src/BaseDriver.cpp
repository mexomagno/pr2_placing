/*
Servicio de movimiento de la base, de bajo nivel.
Es capaz de desplazar la base y de girarla, sin tener en cuenta los obstáculos del camino.

Recibe: float32 distancia.
        float32 angulo.  Si distancia es 0, se rota base en eje Z.
        string frame_id indicando 
Retorna: memoria/ErrorMsg


TODO:
    - Terminar ejecución si nodo que envía la orden muere en plena ejecución.

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
const float TWIST_PONDERATOR = 0.25;
const double PI = 3.1415;
// VARIABLES GLOBALES
vector<string> errors(4);
bool store_odom = false;
tf::Vector3 last_position(0,0,0);
// Variables auxiliares

// METODOS
void signalHandler( int signum ){
    ROS_INFO("Recibí CTRL+C. Terminando...");
    ros::spinOnce();
    ros::shutdown();
    exit (EXIT_SUCCESS);
}
int turn(double angle){
    return 0;
}
int travel(double distance, double angle){
    ROS_INFO("Desplazar distancia=%f, angulo=%f",distance,angle);
    // tf::TransformListener tf_listener;
    // tf::StampedTransform start_tf, current_tf;
    ros::Rate rate(LOOP_FREQ);
    store_odom = true;
    rate.sleep();
    ros::spinOnce();
    tf::Vector3 start_position = last_position;
    /*if (not tf_listener.waitForTransform(ROBOT_FRAME, WORLD_FRAME, ros::Time(0), ros::Duration(WAIT_TF_TIMEOUT))){
        ROS_ERROR("Transformación no pudo ser obtenida en %f s",WAIT_TF_TIMEOUT);
        return 2;
    }*/
    // Mensaje a enviar a la base
    geometry_msgs::Twist base_cmd;
    // ****** Calcular comando a enviar
    tf::Vector3 itongo(distance,0,0),zaxis(0,0,1);
    // Rotar vector unitario
    itongo = itongo.rotate(zaxis, angle);
    ROS_INFO("Vector (%f,0,0) rotado en %f es (%f,%f,%f)",distance,angle,itongo.x(), itongo.y(), itongo.z());
    // Guardar en Twist
    base_cmd.linear.x = itongo.x()*TWIST_PONDERATOR;
    base_cmd.linear.y = itongo.y()*TWIST_PONDERATOR;
    base_cmd.linear.z = 0   ;//itongo.z()*TWIST_PONDERATOR;
    bool done = false;
    double real_distance = 0;
    double traveled_dist;
    while (!done and ros::ok()){
        ROS_INFO("Ciclo");
        // Enviar comando (ponderado)
        base_cmd_pub.publish(base_cmd);
        rate.sleep();
        ros::spinOnce();
        traveled_dist = start_position.distance(last_position);
        ROS_INFO("Traveled_dist = %f",traveled_dist);
        if (traveled_dist > (distance<0 ? -distance:distance)) done = true;
        // Obtener transformación actual
       /* try{
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
        ROS_INFO("Avanzamos %f hasta ahora (paso de %f)",real_distance, traveled_dist);
        if (traveled_dist > distance) done = true;*/
    }
    store_odom = false;
    if (done){
        ROS_INFO("Exito. Desplazamiento real: %fm", traveled_dist);
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
    ROS_INFO("Recibida odometría: (%f,%f,%f)",odom->pose.pose.position.x,odom->pose.pose.position.y,odom->pose.pose.position.z);
    // Actualizar última pose
    last_position.setX(odom->pose.pose.position.x);
    last_position.setY(odom->pose.pose.position.y);
    last_position.setZ(odom->pose.pose.position.z);
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