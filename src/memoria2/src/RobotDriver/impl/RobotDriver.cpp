#include "../RobotDriver.h"
// Constructor y destructor
RobotDriver::RobotDriver(){
	ROS_DEBUG("RobotDriver: Creando un RobotHeadDriver");
    this->head = new RobotHeadDriver();
    ROS_DEBUG("RobotDriver: Creando un RobotBaseDriver");
    this->base = new RobotBaseDriver();
    ROS_DEBUG("RobotDriver: Creando un RobotGripperDriver para cada gripper");
    this->lgripper = new RobotGripperDriver("l");
    this->rgripper = new RobotGripperDriver("r");
}
RobotDriver::~RobotDriver(){
	ROS_DEBUG("RobotDriver: Borrando RobotHeadDriver");
	delete this->head;
	ROS_DEBUG("RobotDriver: Borrando RobotBaseDriver");
	delete this->base;
	ROS_DEBUG("RobotDriver: Destruyendo ambos RobotGripperDrivers");
	delete this->lgripper;
	delete this->rgripper;
}