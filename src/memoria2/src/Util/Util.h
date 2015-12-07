#ifndef UTIL_H
#define UTIL_H

#include <string>
#include <geometry_msgs/Quaternion.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/centroid.h>
#include <tf/transform_listener.h>

using namespace std;
using namespace pcl;

class Util{
    public:
        Util();
        // CONSTANTES PARA TODO EL QUE IMPORTE ESTA CLASE
        // Frames
        static const string BASE_FRAME;
        static const string ODOM_FRAME;
        static const string CAMERA_FRAME;
        static const string KINECT_FRAME;

        // Tópicos
        static const string KINECT_TOPIC;
        static const string KINECT_TOPIC_SELF_FILTERED;
        static const string GRIPPER_GOAL_TOPIC_SUFFIX;
        static const string GRIPPER_STATUS_TOPIC_SUFFIX;
        static const string BASE_CONTROLLER_TOPIC;
        static const string ODOM_TOPIC;

        // Otros
        static const float PI;
        static const float SUBSAMPLE_LEAFSIZE;
        static const float DEFAULT_DESIRED_PITCH;
        static const float KINECT_STABILIZE_TIME;
        // Métodos
        static float toRad(float grad);
        static float toGrad(float rad);
        static geometry_msgs::Quaternion coefsToQuaternionMsg(float a, float b, float c);
        // Operaciones con nubes de puntos
        static PointCloud<PointXYZ>::Ptr subsampleCloud(PointCloud<PointXYZ>::Ptr cloud_in, float leafsize);
        // static scanGripper(char which);
        static bool searchPlacingSurface(PointCloud<PointXYZ>::Ptr cloud_in, PointCloud<PointXYZ>::Ptr &cloud_out, float min_height, float max_height, float inclination);
        static geometry_msgs::Point getCloudCentroid(PointCloud<PointXYZ>::Ptr cloud_in);
        // 
    protected:
    private:
};

#endif // UTIL_H
