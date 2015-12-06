#ifndef ROBOTGRIPPERDRIVER_H
#define ROBOTGRIPPERDRIVER_H
#include "Util.h"
#include <string>
#include <ros/ros.h>
#include <pr2_controllers_msgs/Pr2GripperCommandActionGoal.h>
#include <pr2_controllers_msgs/Pr2GripperCommandActionResult.h>


using namespace std;

class RobotGripperDriver{
	ros::NodeHandle *nh_;
	ros::Subscriber gripper_status_;
	ros::Publisher gripper_goal_;
	char which_;
    public:
        RobotGripperDriver(const string which);
        ~RobotGripperDriver();
        bool setOpening(float opening, float max_effort);
        string getWhich();
    protected:
    private:
};

#endif // ROBOTGRIPPERDRIVER_H
