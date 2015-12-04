#include "../RobotHeadDriver.h"
// CONSTANTES
const string POINT_HEAD_CONTROLLER = "/head_traj_controller/point_head_action";
const float HEAD_MAX_VELOCITY = 0.8;
const float HEAD_MIN_DURATION = 0.1;
const float HEAD_TIMEOUT = 15;
const float WAIT_CONTROLLER_TIMEOUT = 5.0;


RobotHeadDriver::RobotHeadDriver(){
    // Crear cliente de movimiento
    ROS_DEBUG("RobotHeadDriver: Creando nuevo objeto RobotHeadDriver");
    ROS_DEBUG("RobotHeadDriver: Creando cliente para servicio POINT_HEAD_CONTROLLER");
    this->phc_ = new PointHeadClient(POINT_HEAD_CONTROLLER, true);
    ROS_DEBUG("RobotHeadDriver: Esperando servicio cada %f s", WAIT_CONTROLLER_TIMEOUT);
    while (!this->phc_->waitForServer(ros::Duration(WAIT_CONTROLLER_TIMEOUT))){
        ROS_DEBUG("RobotHeadDriver: Esperando al point_head_action_server...");
    }
    ROS_DEBUG("RobotHeadDriver: Mirando hacia el frente");
    // Mirar hacia el frente
    bool res = this->lookAt(Util::BASE_FRAME,10,0,1.5);
    ROS_DEBUG("Ya se miro al frente. Resultado: %s", (res ? "true" : "false"));
}
bool RobotHeadDriver::lookAt(string frame_id, double x, double y, double z){
    geometry_msgs::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    if (not sendLookAt(frame_id, point)){
        return false;
    }
    this->lastpoint_ = point;
    return true;
}
bool RobotHeadDriver::rotate(string frame_id, double rad){
    // Rotación NO es relativa
    // Rotar respecto a Z
    /*
    Rz = | cosW -sinW 0 |
         | sinW cosW  0 |
         |   0    0   1 |
    */
    double modulo = sqrt(this->lastpoint_.x*this->lastpoint_.x + this->lastpoint_.y*this->lastpoint_.y);
    double x = modulo*cos(rad);
    double y = modulo*sin(rad);
    double z = this->lastpoint_.z;
    // Lo siguiente era para poder rotar en Pitch también pero no funcionaba
    // x = x*cos(0); //+ z*sin(0);
    // z = z*cos(0); //x*-sin(0)
    this->lookAt(frame_id, x, y, z);
}

// PRIVATE:
bool RobotHeadDriver::sendLookAt(string frame_id, geometry_msgs::Point p){
    pr2_controllers_msgs::PointHeadGoal goal;
    goal.target.header.frame_id = frame_id;
    goal.target.point = p;
    goal.pointing_frame = Util::KINECT_FRAME;
    goal.pointing_axis.x = 0;
    goal.pointing_axis.y = 0;
    goal.pointing_axis.z = 1;
    goal.max_velocity = HEAD_MAX_VELOCITY;
    goal.min_duration = ros::Duration(HEAD_MIN_DURATION);
    ROS_DEBUG("RobotHeadDriver: Enviando Goal al robot...");
    phc_->sendGoal(goal);
    ROS_DEBUG("RobotHeadDriver: Enviado!");
    if (not phc_->waitForResult(ros::Duration(HEAD_TIMEOUT))){
        ROS_ERROR("RobotHeadDriver: Timeout antes de llegar a pose final");
        return false;
    }
    ROS_DEBUG("RobotHeadDriver: Respuesta del envío fue True");
    return true;
}
