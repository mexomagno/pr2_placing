/*
Este programa permite al robot mover su base según una pose fija.
Es "tonto", osea, tan solo se dedica a mover la base sin pensar en los obstáculos que puedan haber.

Importante es saber que sí existe una forma de más alto nivel para  moverse.    

Algoritmo:
    - Paralelamente,  Actualizar siempre la posición del robot según el odómetro (tópico "/robot_pose_ekf/odom_combined")
    - Mover hacia adelante
    - Mover hacia atrás

    Para mover:
        - Calcular vector desplazamiento
        - Calcular velocidades y tiempo necesario para recorrer distancia
        - Recorrerla
*/
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl/point_types.h>
//#include <tf/transform_listener.h>
// Para usar <<
#include <iostream>

const double MAX_LINEAR_V = 0.5;
const double MAX_ANGULAR_V = 0.5;

ros::Publisher twist_pub;
ros::Subscriber odom_sub;
geometry_msgs::PoseWithCovarianceStamped odom_pose_w;
void moveTo(double x, double y){
    std::cout << "Ir a " << x << "," << y << std::endl;
    std::string frame_id = "odom_combined";
    ros::spinOnce();
/*geometry_msgs/Vector3 linear
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3 angular
  float64 x
  float64 y
  float64 z
*/
    // calcular vector desplazamiento
    Eigen::Vector2f pose_i,pose_f,despl;
    // Asumimos 2 dimensiones
    // pose_i(0)=odom_pose_w.pose.pose.position.x;
    // pose_i(1)=odom_pose_w.pose.pose.position.y;
    // pose_f(0)=x;
    // pose_f(1)=y;
    // despl = pose_f - pose_i;
    // std::cout << "Desplazamiento: " << despl << std::endl;
    // Notar que recibe momentums, no posiciones. Hay que calcular.
    geometry_msgs::Twist goal;
    /*goal.linear.x = 0.05;
    goal.linear.y = 0.05;
    goal.linear.z = 0;*/
    // mover
    //  1) Girar mientras la orientación del robot sea distinta del desplazamiento.
    // while (odom_pose_w.pose.pose.orientation.getYaw() < )
    //     goal.angular.z = MAX_ANGULAR_V;
    //     twist_pub.publish(goal);
    //     ros::spinOnce();
}
// Frecuencia de actualización de pose odom
const float ODOM_UPDATE_FREQ = 10;
//double last_tstamp = 0;
void updateOdomPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose){
    // Obtener timestamp actual
    //double tstamp = (pose->header.stamp).nsec; // segundos
    //double deltat = tstamp - last_tstamp;
    //printf("deltat = %f\n",deltat);
    //if (deltat > 1.0/ODOM_UPDATE_FREQ){
    odom_pose_w = *pose;
    std::cout << "Pose: " << odom_pose_w.pose.pose.position.x << "," << odom_pose_w.pose.pose.position.y <<  std::endl;
    //    last_tstamp = tstamp;
    //}
}
int main(int argc, char **argv){
    ros::init(argc,argv, "movimiento_basico");
    ros::NodeHandle nh;
    ros::Rate loop_rate(0.2);
    // Publisher
    twist_pub = nh.advertise<geometry_msgs::Twist> ("/base_controller/command",1);
    // Subscriber
    odom_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped> ("/robot_pose_ekf/odom_combined",1,updateOdomPose);

    //std::string source_frame = "/base_footprint", target_frame = "/odom_combined";
    //while (last_tstamp = 0){
        // esperar que haya alguna pose del odómetro
    //    loop_rate.sleep();
    //}

    // ros::spin();
    //geometry_msgs::Pose odom_pose;
    while (ros::ok()){
        // Obtener posición del robot según odom_combined
     //   odom_pose = odom_pose_w.pose.pose;
        // Mostrar en consola
        moveTo(1,2);
        //ros::spinOnce();
        loop_rate.sleep();
    }


    return EXIT_SUCCESS;
}