#ifndef ROBOTSENSORS_H
#define ROBOTSENSORS_H

#include <ros/ros.h>
#include "../../Util/Util.h"
#include "Kinect.h"

//#include <sensor_msgs/PointCloud2.h>
class RobotSensors{
	public:
		Kinect *kinect;

		// Constructor y Destructor
		RobotSensors();
		~RobotSensors();

	private:
};


#endif // ROBOTSENSORS_H