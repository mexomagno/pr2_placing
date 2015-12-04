#ifndef ROBOTBASEDRIVER_H
#define ROBOTBASEDRIVER_H
#include "Util.h"
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

using namespace std;

class RobotBaseDriver
{
	ros::NodeHandle *nh_;

    public:
    	// Constructor y Destructor
        RobotBaseDriver();
        ~RobotBaseDriver();
        // Métodos
        bool goToPose(geometry_msgs::PoseStamped pose);
    protected:
    private:
    	bool turn(float angle);
    	bool travel(float distance, float angle);
    	// void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
};

#endif // ROBOTBASEDRIVER_H
