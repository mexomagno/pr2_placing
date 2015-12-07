#include "../RobotSensors.h"

RobotSensors::RobotSensors(){
	// Iniciar Kinect
	ROS_DEBUG("RobotSensors: Creando sensor Kinect");
	this->kinect = new Kinect();
	ROS_DEBUG("RobotSensors: Sensor Kinect creado");
}
RobotSensors::~RobotSensors(){
	ROS_DEBUG("RobotSensors: Destruyendo sensor Kinect");
	delete this->kinect;
}