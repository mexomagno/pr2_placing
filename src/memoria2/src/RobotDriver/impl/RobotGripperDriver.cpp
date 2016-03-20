#include "../RobotGripperDriver.h"

// CONSTANTES
const float MAX_EFFORT = 1000;
const float RESEND_DELAY = 0.1;
const float MAX_OPENING = 0.085;
const float MIN_OPENING = 0;
const int GO_TO_POSE_RETRIES = 2;
const float ELSEWHERE_POSITION[] = {0, 0.4, -0.62}; 
const float ELSEWHERE_ORIENTATION[] = {Util::PI/2.0, Util::PI/2.0, 0}; 


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
    ROS_INFO("RobotGripperDriver: Creando gripper %c", this->which_);
    this->nh_      = new ros::NodeHandle;
    GOAL_TOPIC     = which + Util::GRIPPER_GOAL_TOPIC_SUFFIX;
    STATUS_TOPIC   = which + Util::GRIPPER_STATUS_TOPIC_SUFFIX;
    gripper_goal_   = this->nh_->advertise<pr2_controllers_msgs::Pr2GripperCommandActionGoal>(GOAL_TOPIC, 1);
    gripper_status_ = this->nh_->subscribe(STATUS_TOPIC, 1, statusCallback);
    
    string group_s = (this->which_ == 'l' ? "left_arm" : "right_arm");
    ROS_INFO("RobotGripperDriver: Inicializando grupo '%s'", group_s.c_str());
    // moveit::planning_interface::MoveGroup group(group_s.c_str());
    this->moveit_group_ = new moveit::planning_interface::MoveGroup(group_s.c_str());
    ROS_INFO("RobotGripperDriver: Creando e iniciando spinner");
    this->spinner_ = new ros::AsyncSpinner(1);
    this->spinner_->start();
    ROS_INFO("RobotGripperDriver: grupo iniciado");
}
RobotGripperDriver::~RobotGripperDriver(){
    ROS_INFO("RobotGripperDriver: Destruyendo nodehandle");
    delete this->nh_;
    ROS_INFO("RobotGripperDriver: Destruyendo moveit_group");
    delete this->moveit_group_;
    ROS_INFO("RobotGripperDriver: Deteniendo y destruyendo spinner");
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
	ROS_INFO("RobotGripperDriver: Seteando %s gripper apertura %f",this->getWhich().c_str(), opening);
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
	ROS_INFO("RobotGripperDriver: %s gripper movido", this->getWhich().c_str());
	return true;
}
bool RobotGripperDriver::setOpeningNonBlocking(float opening, float max_effort = MAX_EFFORT){
    ROS_INFO("RobotGripperDriver: Seteando %s gripper apertura %f", this->getWhich().c_str(), opening);
    ROS_WARN("RobotGripperDriver: No se esperará feedback");
    float position = (MAX_OPENING - MIN_OPENING)*opening + MIN_OPENING;
    pr2_conrollers_msgs::Pr2GripperCommandActionGoal action_goal;
    action_goal.goal.command.max_effort = (max_effort < MAX_EFFORT ? max_effort : MAX_EFFORT);
    while (gripper_goal_.getNumSubscribers() == 0)
        ros::Duration(0.05).sleep();
    gripper_goal_.publish(action_goal);
    gripper_goal_.publish(action_goal);
    gripper_goal_.publish(action_goal);
    ROS_INFO("RobotGripperDriver: Comando correctamente enviado a gripper %s", this->getWhich().c_str());
}
string RobotGripperDriver::getWhich(){
	switch (this->which_){
		case 'l': return "left";
		case 'r': return "right";
		default: ROS_ERROR("RobotGripperDriver: Esto es un error que no debiera salir absolutamente nunca");
				return "error";
	}
}
geometry_msgs::PoseStamped RobotGripperDriver::getCurrentPose(){
    return this->moveit_group_->getCurrentPose();
}
bool RobotGripperDriver::goToPose(geometry_msgs::PoseStamped pose_in){
    geometry_msgs::PoseStamped pose;
    if (pose_in.header.frame_id != Util::ODOM_FRAME){
        ROS_WARN("RobotGripperDriver: Se recibe pose respecto a frame '%s'. Transformando...", pose_in.header.frame_id.c_str());
        Eigen::Matrix4f tf = Util::getTransformation(pose_in.header.frame_id, Util::ODOM_FRAME);
        pose.pose = Util::transformPose(pose_in.pose, tf);
        pose.header.frame_id = Util::ODOM_FRAME;
    }
    else
        pose=pose_in;
	this->moveit_group_->setPoseTarget(pose);
    ROS_INFO("RobotGripperDriver: Planeando para pose (%.2f, %.2f, %.2f) - (%.2f, %.2f, %.2f, %.2f)", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
    bool plan_done;
    for (int i=0; i<GO_TO_POSE_RETRIES; i++){
        ROS_INFO("RobotGripperDriver: Intento %d de %d de ir a pose", i+1, GO_TO_POSE_RETRIES);
        plan_done = this->moveit_group_->plan(this->planner_);
        if (plan_done){
            ROS_INFO("RobotGripperDriver: Plan EXITOSO");
            for (int j=0; j<GO_TO_POSE_RETRIES; j++){
                ROS_INFO("Intento %d de %d de ejecutarlo...", j+1, GO_TO_POSE_RETRIES);
                bool move_done = this->moveit_group_->move();
                if (move_done){
                    ROS_INFO("RobotGripperDriver: Plan ejecutado con éxito.");
                    return move_done;
                }
                ROS_WARN("Ocurrió un problema al ejecutar el plan. %s", (j< GO_TO_POSE_RETRIES - 1 ? " Reintentando..." : ""));
            }
            break;
        }
        ROS_WARN("RobotGripperDriver: Plan FALLIDO %s", (i < GO_TO_POSE_RETRIES - 1 ? ". Reintentando..." : ""));
    }
    ROS_ERROR("RobotGripperDriver: No se pudo ir a la pose después de %d intentos", GO_TO_POSE_RETRIES);
    return false;
}
/**
 * moveElsewhere mueve el gripper a un lugar donde no moleste. Por ahora se define esta pose como al lado del robot, abajo.
 * @return [description]
 */
bool RobotGripperDriver::moveElsewhere(){
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = Util::TORSO_FRAME;
    pose.pose.position.x = ELSEWHERE_POSITION[0];
    pose.pose.position.y = ELSEWHERE_POSITION[1] * (which_ == 'l' ? 1 : -1);
    pose.pose.position.z = ELSEWHERE_POSITION[2];
    pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(ELSEWHERE_ORIENTATION[0], ELSEWHERE_ORIENTATION[1], ELSEWHERE_ORIENTATION[2]);
    return goToPose(pose);
}
/**
 * TODO:
 * 		- Implementar timeout en setOpening.
 *
 * DONE:
 *      - Implementer goToPose dependiente del frame
 */