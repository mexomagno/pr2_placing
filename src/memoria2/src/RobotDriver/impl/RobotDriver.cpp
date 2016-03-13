#include "../RobotDriver.h"
// Constructor y destructor
RobotDriver::RobotDriver(){
	ROS_INFO("RobotDriver: Creando un RobotHeadDriver");
    this->head = new RobotHeadDriver();
    ROS_INFO("RobotDriver: Creando un RobotBaseDriver");
    this->base = new RobotBaseDriver();
    ROS_INFO("RobotDriver: Creando un RobotGripperDriver para cada gripper");
    this->lgripper = new RobotGripperDriver("l");
    this->rgripper = new RobotGripperDriver("r");
    ROS_INFO("RobotDriver: Creando sensores");
    this->sensors = new RobotSensors();
    
}
RobotDriver::~RobotDriver(){
	ROS_INFO("RobotDriver: Borrando RobotHeadDriver");
	delete this->head;
	ROS_INFO("RobotDriver: Borrando RobotBaseDriver");
	delete this->base;
	ROS_INFO("RobotDriver: Destruyendo ambos RobotGripperDrivers");
	delete this->lgripper;
	delete this->rgripper;
	ROS_INFO("RobotDriver: Destruyendo sensores");
	delete this->sensors;
}