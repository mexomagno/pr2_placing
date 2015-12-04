#include "../Util.h"

// PUBLIC
// CONSTANTES
// Frames
const string Util::BASE_FRAME                  = "/base_footprint";
const string Util::ODOM_FRAME = "/odom_combined";
const string Util::CAMERA_FRAME                = "/high_def_frame";
const string Util::KINECT_FRAME                = "/head_mount_kinect_ir_optical_frame";

// Tópicos
const string Util::KINECT_TOPIC                = "head_mount_kinect/depth_registered/points";
const string Util::GRIPPER_GOAL_TOPIC_SUFFIX   = "_gripper_controller/gripper_action/goal";
const string Util::GRIPPER_STATUS_TOPIC_SUFFIX = "_gripper_controller/gripper_action/result";
const string Util::BASE_CONTROLLER_TOPIC       = "/base_controller/command";
const string Util::ODOM_TOPIC = "/base_odometry/odom";

// Otros
const float Util::PI  = 3.1416;

// MÉTODOS
Util::Util(){}
float Util::toGrad(float rad){
	return rad*180.0/PI;
}
float Util::toRad(float grad){
	return grad*PI/180.0;
}
