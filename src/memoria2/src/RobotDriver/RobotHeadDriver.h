#ifndef ROBOTHEADDRIVER_H
#define ROBOTHEADDRIVER_H
#include "../Util/Util.h"
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pr2_controllers_msgs/PointHeadGoal.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <actionlib/client/simple_action_client.h>

using namespace std;
using namespace pcl;

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

class RobotHeadDriver{
    PointHeadClient *phc_;
    geometry_msgs::Point lastpoint_;
    public:
        // Métodos
        // Constructor y destructor
        RobotHeadDriver();
        ~RobotHeadDriver();
        bool lookAt(string frame_id, double x, double y, double z);
        bool rotate(string frame_id, double rad);

    protected:
    private:
        bool sendLookAt(string frame_id, geometry_msgs::Point p);
};

#endif // ROBOTHEADDRIVER_H
