#ifndef UTIL_H
#define UTIL_H

#include <string>
#include <vector>
#include <geometry_msgs/Quaternion.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/centroid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_listener.h>
#include "../RobotDriver/RobotDriver.h"

using namespace std;
using namespace pcl;
// Para uso en gripperfilter
struct box{
    float center[3];
    float size[3];
};
typedef struct box Box;
// FIN gripperfilter

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
        static const float ROBOT_FRONT_MARGIN;
        // Métodos
        static float toRad(float grad);
        static float toGrad(float rad);
        static geometry_msgs::Quaternion coefsToQuaternionMsg(float a, float b, float c);
        // Operaciones con nubes de puntos
        static PointCloud<PointXYZ>::Ptr subsampleCloud(PointCloud<PointXYZ>::Ptr cloud_in, float leafsize);
        static geometry_msgs::Point getCloudCentroid(PointCloud<PointXYZ>::Ptr cloud_in);
        static void getClosestPoint(PointCloud<PointXYZ>::Ptr cloud, geometry_msgs::PointStamped &closest_point, float &closest_point_distance);
        static Eigen::Matrix4f getTransformation(string orig_frame, string target_frame);
        // Utilidades específicas
        static bool searchPlacingSurface(PointCloud<PointXYZ>::Ptr cloud_in, PointCloud<PointXYZ>::Ptr &cloud_out, float min_height, float max_height, float inclination);
        static PointCloud<PointXYZ>::Ptr scanGripper(RobotDriver r_driver, char which);
    protected:
    private:
        static void gripperFilter(PointCloud<PointXYZ>::Ptr cloud_in, PointCloud<PointXYZ>::Ptr &object_out, PointCloud<PointXYZ>::Ptr &gripper_out);
};      static bool isPointInsideBox(PointXYZ p, Box box);

#endif // UTIL_H
