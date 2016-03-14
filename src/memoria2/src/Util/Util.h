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
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/centroid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/PolygonMesh.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <tf/transform_listener.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
// #include "../Polymesh/Polymesh.h"
// #include "../RobotDriver/RobotDriver.h"

using namespace std;
using namespace pcl;
// Para uso en gripperfilter
struct box{
    float center[3];
    float size[3];
};
typedef struct box Box;
// FIN gripperfilter

// Forward declaration de la clase RobotDriver
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
        static const int   FIND_SURFACE_POSE_RETRYS;
        static const float SURFACE_REFINING_THRESHOLD;
        static const int   SURFACE_REFINING_ITERATIONS;
        static const float active_gripper_starting_position[];
        static const float active_gripper_starting_orientation[];
        // Gripper scanner
        static const float SCAN_ROLL_DELTA;
        static const string GRIPPER_FRAME_SUFFIX;
        static const float scan_position[];
        static const float scan_orientation[];
        static const float tuck_position[];
        static const float tuck_orientation[];
        static const float GRIPPER_STABILIZE_TIME;
        static const float SCAN_PASSTHROUGH_Z;
        static const float SCAN_LEAFSIZE;
        // GetPlacingPose
        static const float PATCH_ANGLE_THRESHOLD;
        static const float PLACING_Z_MARGIN;
        static const float PLACING_BACKOFF_DISTANCE;
        // METODOS
        // Conversiones
        static float toRad(float grad);
        static float toGrad(float rad);
        static geometry_msgs::Quaternion coefsToQuaternionMsg(float a, float b, float c);
        static float angleBetweenVectors(float x1, float y1, float z1, float x2, float y2, float z2);
        static Eigen::Vector3f quaternionMsgToVector(geometry_msgs::Quaternion ros_q);
        static Eigen::Quaternionf eigenVectorToQuaternion(Eigen::Vector3f v);
        // Operaciones con nubes de puntos
        static PointCloud<PointXYZ>::Ptr subsampleCloud(PointCloud<PointXYZ>::Ptr cloud_in, float leafsize);
        static geometry_msgs::Point getCloudCentroid(PointCloud<PointXYZ>::Ptr cloud_in);
        static void getClosestPoint(PointCloud<PointXYZ>::Ptr cloud, geometry_msgs::PointStamped &closest_point, float &closest_point_distance);
        static Eigen::Matrix4f getTransformation(string orig_frame, string target_frame);
        static PolygonMesh getConvexHull(PointCloud<PointXYZ>::Ptr cloud);
        // Transformaciones
        static Eigen::Vector3f transformVector(Eigen::Vector3f vector_in, Eigen::Matrix4f transf);
        static geometry_msgs::Point transformPoint(geometry_msgs::Point point_in, Eigen::Matrix4f transf);
        static geometry_msgs::Pose transformPose(geometry_msgs::Pose pose_in, Eigen::Matrix4f transf);
        static Eigen::Matrix3f getRotationBetweenVectors(Eigen::Vector3f v1, Eigen::Vector3f v2);
        static Eigen::Quaternionf getQuaternionBetweenVectors(Eigen::Vector3f vini, Eigen::Vector3f vend);
        // static Eigen::Matrix4f getTransformBetweenPoses(geometry_msgs::Pose pose_ini, geometry_msgs::Pose pose_end);
        // Utilidades específicas
        static bool searchPlacingSurface(PointCloud<PointXYZ>::Ptr cloud_in, PointCloud<PointXYZ>::Ptr &cloud_out, geometry_msgs::PoseStamped &surface_normal, geometry_msgs::PointStamped &surface_centroid, float min_height, float max_height, float inclination);
        static bool gripperFilter(PointCloud<PointXYZ>::Ptr cloud_in, PointCloud<PointXYZ>::Ptr &object_out, PointCloud<PointXYZ>::Ptr &gripper_out);
        static PointIndices::Ptr getFactiblePlacingPointsIndices(PointCloud<PointXYZ>::Ptr cloud_in, geometry_msgs::PoseStamped surface_normal_pose, float base_area);
        // static bool getStablePose(PointCloud<PointXYZ>::Ptr object_pc, PointCloud<PointXYZ>::Ptr gripper_pc, geometry_msgs::PoseStamped &pose_out);
        static bool isPointCloudCutByPlane(PointCloud<PointXYZ>::Ptr cloud, ModelCoefficients::Ptr coefs, PointXYZ p_plane);

        static void disableGripperCollisions(char which_gripper, bool disable, ros::Publisher &attached_object_pub, ros::Publisher &collision_object_pub);
    protected:
    private:
        static bool isPointInsideBox(PointXYZ p, Box box);

};

#endif // UTIL_H
