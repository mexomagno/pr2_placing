#include "../RobotDriver.h"
// Constructor y destructor
RobotDriver::RobotDriver(){
	ROS_INFO("RobotDriver: Creando un RobotHeadDriver");
    this->head = new RobotHeadDriver();
    ROS_INFO("RobotDriver: Creando un RobotBaseDriver");
    this->base = new RobotBaseDriver();
}
RobotDriver::~RobotDriver(){
	ROS_DEBUG("RobotDriver: Borrando RobotHeadDriver");
	delete head;
}