#include <string>
#include <csignal>
#include <vector>
#include <ros/ros.h>
#include <pr2_controllers_msgs/Pr2GripperCommandActionGoal.h>
#include <pr2_controllers_msgs/Pr2GripperCommandActionResult.h>
#include <memoria/GripperDriver.h>
#include <memoria/ErrorMsg.h>

using namespace std;

/*class Gripper{
private:
  static const int MSG_RESEND = 50;
  string GRIPPER_CMD_TOPIC;
  string GRIPPER_CMD_STATUS_TOPIC;
  ros::Publisher gripper_pub_;
  ros::Subscriber gripper_status_sub_;
  bool reached_goal_;
  ros::NodeHandle nh_;
  void gripperCallback(const pr2_controllers_msgs::Pr2GripperCommandActionResult::Ptr& gripper_status){
    ROS_INFO("Callback");
    reached_goal_ = gripper_status->result.reached_goal;
  }
public:
  Gripper(string lr, ros::NodeHandle &nh){
    ROS_INFO("Creando Gripper %s",lr.c_str());
    // Valida tipo de gripper
    string lrgripper (lr);
    if (lrgripper.compare("r") == 0 or lrgripper.compare("R") == 0){
      lrgripper = "/r";
    }
    else if (lrgripper.compare("l") == 0 or lrgripper.compare("L") == 0){
      lrgripper = "/l";
    }
    else {
      fprintf(stderr, "Debe ingresar letras 'l' o 'r'");
      ros::shutdown();
      exit(1);
    }
    nh_=nh;
    // Edita constante según gripper deseado
    GRIPPER_CMD_TOPIC = lrgripper+"_gripper_controller/gripper_action/goal";
    GRIPPER_CMD_STATUS_TOPIC = lrgripper+"_gripper_controller/gripper_action/result";
    reached_goal_ = false;
    //crear publicador
    ROS_INFO("Publicando en %s",GRIPPER_CMD_TOPIC.c_str());
    //gripper_pub_ = nh_.advertise<pr2_controllers_msgs::Pr2GripperCommandActionGoal>(GRIPPER_CMD_TOPIC,1);
    gripper_pub_ = nh_.advertise<pr2_controllers_msgs::Pr2GripperCommandActionGoal>(GRIPPER_CMD_TOPIC,1);
    // Dormir un poco
    ros::Duration(0.1).sleep();
    //crea suscriptor a tópico de status
    // gripper_status_sub_ = nh.subscribe<pr2_controllers_msgs::Pr2GripperCommandActionResult>(GRIPPER_CMD_STATUS_TOPIC,1,gripperCallback);
    gripper_status_sub_ = nh.subscribe(GRIPPER_CMD_STATUS_TOPIC,1,&Gripper::gripperCallback, this);
  }
  ~Gripper(){}
  void open(){
    // Publicar mensaje
    customGrip(0.09,-1);
  }
  void close(){
    customGrip(0, -1);
  }
  void customGrip(double position, double max_effort){
    ROS_INFO("Creando mensaje");
    reached_goal_ = false;
    pr2_controllers_msgs::Pr2GripperCommandActionGoal action_goal;
    action_goal.goal.command.position = position;
    action_goal.goal.command.max_effort = max_effort;
    ROS_INFO("Publicando position:%f, max_effort:%f",position,max_effort);
    while (gripper_pub_.getNumSubscribers() == 0)
      ros::Duration(0.05).sleep();
    while (not reached_goal_){
      gripper_pub_.publish(action_goal);
      ros::Duration(0.05).sleep();
      ros::spinOnce();

    }
    ROS_INFO("Publicado");
  }
};




*/
/*
Mucho del comportamiento de los grippers depende de parámetros en el servidor de parámetros de ROS.

/x_gripper_controller/gripper_action_node/stall_timeout : Cantidad de tiempo que debe permaneceer estacionaria para ser considerada como en "stall"
 "                         "             /stall_velocity_threshold : Parámetro usado cuando stall velocity oscila (simulation jitter).

 También se pueden modificar valores del archivo $(rospack find pr2_controller_configuration_gazebo/config/pr2_gripper_controllers.yaml para la simulación)

TODO:
  - Añadir información de posición y effort de gripper en pleno movimiento
  - Añadir funciones no-bloqueantes para abrir y cerrar grippers
*/


// CONSTANTES
const string CMD_TOPIC_SUFFIX = "_gripper_controller/gripper_action/goal";
const string STATUS_TOPIC_SUFFIX = "_gripper_controller/gripper_action/result";
const float MAX_GRIPPER_EFFORT = 10000;
const float RESEND_DELAY = 0.1;
const float MAX_GRIPPER_OPENING = 0.085;
const float MIN_GRIPPER_OPENING = 0;

/*const string LEFT_CMD_TOPIC = "l_gripper_controller/gripper_action/goal";
const string RIGHT_CMD_TOPIC = "r_gripper_controller/gripper_action/goal";
const string LEFT_STATUS_TOPIC = "l_gripper_controller/gripper_action/result";
const string RIGHT_STATUS_TOPIC = "l_gripper_controller/gripper_action/result";
*/
// Variables Globales
ros::Publisher left_pub, right_pub;
bool left_goal_reached = false,right_goal_reached = false;
vector<string> errors(2);
void signalHandler( int signum ){
    ROS_INFO("Recibí CTRL+C. Terminando...");
    //ros::shutdown();
    exit (EXIT_SUCCESS);
}
int setOpening(bool right, double opening, double max_effort){
  // Valor de 0 a 1 se mapea de MIN a MAX seteado
  double position = (MAX_GRIPPER_OPENING - MIN_GRIPPER_OPENING)*opening + MIN_GRIPPER_OPENING;
  // Crear mensaje a enviar
  pr2_controllers_msgs::Pr2GripperCommandActionGoal action_goal;
  action_goal.goal.command.position = position;
  action_goal.goal.command.max_effort = (max_effort != 0 ? (max_effort < MAX_GRIPPER_EFFORT ? max_effort : MAX_GRIPPER_EFFORT) : MAX_GRIPPER_EFFORT);
  if (not right){
    left_goal_reached = false;
    while (left_pub.getNumSubscribers() == 0)
      ros::Duration(0.05).sleep();
    while (not left_goal_reached){
      left_pub.publish(action_goal);
      ros::Duration(RESEND_DELAY).sleep();
      ros::spinOnce();
    }
  }
  else{
    right_goal_reached = false;
    while (right_pub.getNumSubscribers() == 0)
      ros::Duration(0.05).sleep();
    while (not right_goal_reached){
      right_pub.publish(action_goal);
      ros::Duration(RESEND_DELAY).sleep();
      ros::spinOnce();
    }
  }
  ROS_INFO("Griper %s movido", (right ? "derecho" : "izquierdo"));
  return 0;
}
void lGripperCallback(const pr2_controllers_msgs::Pr2GripperCommandActionResult::Ptr &result){
  left_goal_reached = result->result.reached_goal;
}
void rGripperCallback(const pr2_controllers_msgs::Pr2GripperCommandActionResult::Ptr &result){
  right_goal_reached = result->result.reached_goal;
}
bool callback(memoria::GripperDriver::Request &request, memoria::GripperDriver::Response &response){
  ROS_INFO("Recibido Request: Gripper: %s, apertura: %f, max_effort: %f", (request.right ? "right" : "left"), request.opening, request.max_effort);
  int retcode = 0;
  // Enviar instrucción a gripper adecuado
  retcode = setOpening(request.right, request.opening, request.max_effort);
  memoria::ErrorMsg errormsg;
  errormsg.retcode = retcode;
  errormsg.what = errors[errormsg.retcode];
  response.error = errormsg;
  return true;
}
int main(int argc, char** argv){
  errors[0]="";
  errors[1]="Error Desconocido";

  ros::init(argc, argv, "gripper_driver");
  ros::NodeHandle nh;
  signal(SIGINT, signalHandler);
  // Crear publicadores
  left_pub = nh.advertise<pr2_controllers_msgs::Pr2GripperCommandActionGoal>("l"+CMD_TOPIC_SUFFIX,1);
  right_pub = nh.advertise<pr2_controllers_msgs::Pr2GripperCommandActionGoal>("r"+CMD_TOPIC_SUFFIX,1);
  // Crear subscriptores al estado de grippers
  ros::Subscriber left_sub = nh.subscribe("l"+STATUS_TOPIC_SUFFIX,1,lGripperCallback);
  ros::Subscriber right_sub = nh.subscribe("r"+STATUS_TOPIC_SUFFIX,1,rGripperCallback);
  // Crear Servicio
  ros::ServiceServer service = nh.advertiseService("gripper_driver", callback);
  ROS_INFO("Listo para recibir requests en gripper_driver");
  ros::spin();
  return 0;
}