#ifndef ROBOTGRIPPERDRIVER_H
#define ROBOTGRIPPERDRIVER_H
#include "../Util/Util.h"
#include <string>
#include <ros/ros.h>
#include <pr2_controllers_msgs/Pr2GripperCommandActionGoal.h>
#include <pr2_controllers_msgs/Pr2GripperCommandActionResult.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_interface/planning_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>

using namespace std;
using namespace pcl;

class RobotGripperDriver{
	ros::NodeHandle *nh_;
	ros::Subscriber gripper_status_;
	ros::Publisher gripper_goal_;
    moveit::planning_interface::MoveGroup::Plan planner_;
    moveit::planning_interface::MoveGroup *moveit_group_;
    ros::AsyncSpinner *spinner_;
	char which_;
    public:
        RobotGripperDriver(const string which);
        ~RobotGripperDriver();
        bool setOpening(float opening, float max_effort);
        string getWhich();
        bool goToPose(geometry_msgs::PoseStamped pose);
    protected:
    private:
};

#endif // ROBOTGRIPPERDRIVER_H
