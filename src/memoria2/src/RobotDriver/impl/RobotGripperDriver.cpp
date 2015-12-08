#include "../RobotGripperDriver.h"

// CONSTANTES
const float MAX_EFFORT = 1000;
const float RESEND_DELAY = 0.1;
const float MAX_OPENING = 0.085;
const float MIN_OPENING = 0;
string GOAL_TOPIC;
string STATUS_TOPIC;
// VARIABLES GLOBALES
bool goal_reached = false;

void statusCallback(const pr2_controllers_msgs::Pr2GripperCommandActionResult::Ptr &result){
  goal_reached = result->result.reached_goal;
}
RobotGripperDriver::RobotGripperDriver(const string which){
    // Ver qué gripper es
    if (which.compare("l") != 0 and which.compare("r") != 0){
    	ROS_ERROR("RobotGripperDriver: Valor inválido '%s'. Debe ser [l|r]", which.c_str());
    	return;
    }
    this->which_   = which[0];
    ROS_DEBUG("RobotGripperDriver: Creando gripper %c", this->which_);
    this->nh_      = new ros::NodeHandle;
    GOAL_TOPIC     = which + Util::GRIPPER_GOAL_TOPIC_SUFFIX;
    STATUS_TOPIC   = which + Util::GRIPPER_STATUS_TOPIC_SUFFIX;
    gripper_goal_   = this->nh_->advertise<pr2_controllers_msgs::Pr2GripperCommandActionGoal>(GOAL_TOPIC, 1);
    gripper_status_ = this->nh_->subscribe(STATUS_TOPIC, 1, statusCallback);
    
    string group_s = (this->which_ == 'l' ? "left_arm" : "right_arm");
    ROS_DEBUG("RobotGripperDriver: Inicializando grupo '%s'", group_s.c_str());
    // moveit::planning_interface::MoveGroup group(group_s.c_str());
    this->moveit_group_ = new moveit::planning_interface::MoveGroup(group_s.c_str());
    ROS_DEBUG("RobotGripperDriver: Creando e iniciando spinner");
    this->spinner_ = new ros::AsyncSpinner(1);
    this->spinner_->start();
    ROS_DEBUG("RobotGripperDriver: grupo iniciado");
}
RobotGripperDriver::~RobotGripperDriver(){
    ROS_DEBUG("RobotGripperDriver: Destruyendo nodehandle");
    delete this->nh_;
    ROS_DEBUG("RobotGripperDriver: Destruyendo moveit_group");
    delete this->moveit_group_;
    ROS_DEBUG("RobotGripperDriver: Deteniendo y destruyendo spinner");
    this->spinner_->stop();
    delete this->spinner_;
}
/**
 * RobotGripperDriver::setOpening: Setea apertura del gripper.
 * @param  opening    : Setea apertura, valores de 0 a 1
 * @param  max_effort : Máximo esfuerzo. De 0 a MAX_EFFORT.
 * @return            : True en éxito. Por ahora no hay condición para False.
 */
bool RobotGripperDriver::setOpening(float opening, float max_effort = MAX_EFFORT){
	ROS_DEBUG("RobotGripperDriver: Soy el %s gripper, topic: %s", this->getWhich().c_str(), gripper_status_.getTopic().c_str());
	ROS_DEBUG("RobotGripperDriver: Seteando %s gripper apertura %f",this->getWhich().c_str(), opening);
	float position = (MAX_OPENING - MIN_OPENING)*opening + MIN_OPENING;
	pr2_controllers_msgs::Pr2GripperCommandActionGoal action_goal;
	action_goal.goal.command.position = position;
	action_goal.goal.command.max_effort = (max_effort < MAX_EFFORT ? max_effort : MAX_EFFORT);
	goal_reached = false;
	while (gripper_goal_.getNumSubscribers() == 0)
		ros::Duration(0.05).sleep();
	while (not goal_reached){
		gripper_goal_.publish(action_goal);
		ros::Duration(RESEND_DELAY).sleep();
		ros::spinOnce();
	}
	ROS_DEBUG("RobotGripperDriver: %s gripper movido", this->getWhich().c_str());
	return true;
}
string RobotGripperDriver::getWhich(){
	switch (this->which_){
		case 'l': return "left";
		case 'r': return "right";
		default: ROS_ERROR("RobotGripperDriver: Esto es un error que no debiera salir absolutamente nunca");
				return "error";
	}
}
bool RobotGripperDriver::goToPose(geometry_msgs::PoseStamped pose){
	this->moveit_group_->setPoseTarget(pose);
    ROS_DEBUG("RobotGripperDriver: Planeando para pose (%.2f, %.2f, %.2f) - (%.2f, %.2f, %.2f, %.2f)", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
    bool plan_done = this->moveit_group_->plan(this->planner_);
    ROS_DEBUG("RobotGripperDriver: Plan %s", plan_done ? "EXITOSO" : "FALLIDO");
    if (plan_done){
    	ROS_DEBUG("RobotGripperDriver: Ejecutando plan...");
    	this->moveit_group_->move();
    }
    return plan_done;
}
/**
 * TODO:
 * 		- Implementar timeout en setOpening.
 */