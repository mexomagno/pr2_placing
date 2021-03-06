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
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
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
#include "../Polymesh/Polymesh.h"
#include "../PlacingSurface/PlacingSurface.h"
#include <geometric_shapes/shape_operations.h>
// #include <shape_msgs/ShapeMsg.h>
// #include "../RobotDriver/RobotDriver.h"

using namespace std;
using namespace pcl;

#ifndef STRUCT_BOX
#define STRUCT_BOX
// Para uso en gripperfilter
struct box{
    float center[3];
    float size[3];
};
typedef struct box Box;
// FIN gripperfilter
#endif // STRUCT_BOX
#ifndef STRUCT_BOUNDING_BOX
#define STRUCT_BOUNDING_BOX
struct bounding_box{
    PointXYZ min;
    PointXYZ max;
    geometry_msgs::Point position;
    geometry_msgs::Quaternion rotation;
};
typedef struct bounding_box BBOriented;
#endif // STRUCT_BOUNDING_BOX
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
        static const string TORSO_FRAME;
        static const string GRIPPER_FRAME_SUFFIX;
        static const string SHOULDER_FRAME_SUFFIX;

        // Tópicos
        static const string KINECT_TOPIC;
        static const string KINECT_TOPIC_SELF_FILTERED;
        static const string GRIPPER_GOAL_TOPIC_SUFFIX;
        static const string GRIPPER_STATUS_TOPIC_SUFFIX;
        static const string BASE_CONTROLLER_TOPIC;
        static const string ODOM_TOPIC;

        // Strings en general
        static const string COLLISION_BALL_ID;
        static const string COLLISION_MESH_ID;
        static const string COLLISION_SURFACE_ID;
        static const string GRIPPER_JOINT_PREFIX;
        static const string GRIPPER_LINK_PREFIX;
        static const string ARM_ROLL_JOINT_PREFIX;

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
        static const float MOVEIT_PLANNING_TIME;
        static const string MOVEIT_PLANNER;
        // Gripper scanner
        static const float SCAN_ROLL_DELTA;
        static const float scan_position[];
        static const float scan_orientation[];
        static const float tuck_position[];
        static const float tuck_orientation[];
        static const float GRIPPER_STABILIZE_TIME;
        static const float SCAN_PASSTHROUGH_Z;
        static const float SCAN_LEAFSIZE;
        static       float COLLISION_BALL_RADIUS;
        static const float VOXEL_UPDATE_DELAY;
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
        static PolygonMesh getTriangulation(PointCloud<PointXYZ>::Ptr cloud);
        // Transformaciones
        static Eigen::Vector3f transformVector(Eigen::Vector3f vector_in, Eigen::Matrix4f transf);
        static geometry_msgs::Point transformPoint(geometry_msgs::Point point_in, Eigen::Matrix4f transf);
        static geometry_msgs::Quaternion transformOrientation(geometry_msgs::Quaternion q, Eigen::Matrix4f transf);
        static geometry_msgs::Pose transformPose(geometry_msgs::Pose pose_in, Eigen::Matrix4f transf);
        static Eigen::Matrix3f getRotationBetweenVectors(Eigen::Vector3f v1, Eigen::Vector3f v2);
        static Eigen::Quaternionf getQuaternionBetweenVectors(Eigen::Vector3f vini, Eigen::Vector3f vend);
        // Utilidades específicas
        static bool searchPlacingSurface(PointCloud<PointXYZ>::Ptr cloud_in, PointCloud<PointXYZ>::Ptr &cloud_out, geometry_msgs::PoseStamped &surface_normal, geometry_msgs::PointStamped &surface_centroid, float min_height, float max_height, float inclination);
        static bool gripperFilter(PointCloud<PointXYZ>::Ptr cloud_in, PointCloud<PointXYZ>::Ptr &object_out, PointCloud<PointXYZ>::Ptr &gripper_out);
        static PointIndices::Ptr getFactiblePlacingPointsIndices(PointCloud<PointXYZ>::Ptr cloud_in, geometry_msgs::PoseStamped surface_normal_pose, float base_area);
        static bool isPointCloudCutByPlane(PointCloud<PointXYZ>::Ptr cloud, ModelCoefficients::Ptr coefs, PointXYZ p_plane);
        static vector<geometry_msgs::PoseStamped> getPossiblePlacingPoses(char which_gripper, PlacingSurface the_surface, geometry_msgs::PoseStamped stable_pose, geometry_msgs::PoseStamped gripper_current_pose);

        // Collision World
        static void enableDefaultGripperCollisions(ros::Publisher &attached_object_pub, ros::Publisher &collision_object_pub, bool enable, char which_gripper);
        static void attachBoundingBoxToGripper(ros::Publisher &attached_object_pub, ros::Publisher &collision_object_pub, char which_gripper, BBOriented bounding_box);
        static void detachBoundingBoxFromGripper(ros::Publisher &attached_object_pub, ros::Publisher &collision_object_pub, char which_gripper);
        static void attachMeshToGripper(ros::Publisher &attached_object_pub, ros::Publisher &collision_object_pub, char which_arm, Polymesh &object_mesh);
        static void detachMeshFromGripper(ros::Publisher &attached_object_pub, ros::Publisher &collision_object_pub);
        static void addSurfaceAsCollisionObject(ros::Publisher &collision_object_pub, PolygonMesh &surface_mesh);
    protected:
    private:
        static bool isPointInsideBox(PointXYZ p, Box box);
        static void pclPolygonMeshToShapeMsg(PolygonMesh pclmesh, shape_msgs::Mesh &shapemsg);
        static void removeCollisionObjectFromWorld(ros::Publisher &collision_object_pub, moveit_msgs::CollisionObject co);
        static void setAllowedCollisionLinks(moveit_msgs::AttachedCollisionObject &a_co, char which_gripper);
};

#endif // UTIL_H
