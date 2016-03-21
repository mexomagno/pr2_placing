#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "Util/Util.h"

double PI=3.1416;
float toRad(float grad){
    return grad*PI/180.0;
}
Eigen::Matrix4f getTransformation(string frame_ini, string frame_end){
    tf::TransformListener tf_listener;
    tf::StampedTransform stamped_tf;
    ros::Time transform_time = ros::Time(0);
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    if (frame_ini.compare(frame_end) == 0)
        return transformation;
    try{
        ROS_INFO("UTIL: Esperando transformacion disponible...");
        if (not tf_listener.waitForTransform(frame_end, frame_ini, transform_time, ros::Duration(1))){
            ROS_ERROR("UTIL: Transformacion no pudo ser obtenida antes del timeout (%fs)", 1.0);
            return transformation;
        }
        ROS_INFO("UTIL: Guardando transformacion");
        tf_listener.lookupTransform(frame_end, frame_ini, ros::Time(0), stamped_tf);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("UTIL: Excepcion al obtener transformacion: %s", ex.what());
        return transformation;
    }
    // tf::Matrix3x3 rotation = stamped_tf.getBasis();
    tf::Matrix3x3 rotation (stamped_tf.getRotation());
    tf::Vector3 translation = stamped_tf.getOrigin();
    Eigen::Matrix3f upleft3x3;
    upleft3x3 << rotation[0][0], rotation[0][1], rotation[0][2],
                 rotation[1][0], rotation[1][1], rotation[1][2],
                 rotation[2][0], rotation[2][1], rotation[2][2];
    // cout << "Rotation: \n" << upleft3x3 << endl;
    // cout << "Translation: \n" << translation[0] << "    \t" << translation[1] << "    \t" << translation[2] << endl; 
    transformation.block<3,3>(0,0) = upleft3x3;
    transformation.col(3) = Eigen::Vector4f(translation[0], translation[1], translation[2], 1);
    // cout << "Todo: \n" << transformation << endl;
    return transformation;
    /*  transformation << rotation[0][0], rotation[0][1], rotation[0][2], translation[0],
        rotation[1][0], rotation[1][1], rotation[1][2], translation[1],
        rotation[2][0], rotation[2][1], rotation[2][2], translation[2],
        0, 0, 0, 1;*/
}
geometry_msgs::Quaternion coefsToQuaternionMsg(float a, float b, float c){
    /* 
    Recibe: a, b, c, coeficientes de un plano
    Retorna: Quaternion con la orientación de la normal del plano.
    */
    float vmod=sqrt(a*a+b*b+c*c);
    float pitch = -(1.0*asin(1.0*c/vmod));
    float yaw = (a==0?toRad(90):atan(b/a)) + (a>=0?0:toRad(180));
    return tf::createQuaternionMsgFromRollPitchYaw(0,pitch,yaw);
}
Eigen::Vector3f transformVector(Eigen::Vector3f vector_in, Eigen::Matrix4f transf){
    Eigen::Vector4f vector_in_4f(vector_in[0], vector_in[1], vector_in[2], 1);
    // Obtener rotación desde matriz de transformación
    Eigen::Matrix4f rot;
    rot = transf;
    rot.col(3) = Eigen::Vector4f(0, 0, 0, 1);
    Eigen::Vector4f vector_out_4f = rot*Eigen::Vector4f(vector_in[0], vector_in[1], vector_in[2], 1);
    Eigen::Vector3f vector_out (vector_out_4f[0], vector_out_4f[1], vector_out_4f[2]);
    return vector_out;
}
geometry_msgs::Point transformPoint(geometry_msgs::Point point_in, Eigen::Matrix4f transf){
    geometry_msgs::Point point_out;
    Eigen::Vector4f point_4f;
    point_4f << point_in.x, point_in.y, point_in.z, 1;
    point_4f = transf*point_4f;
    point_out.x = point_4f[0];
    point_out.y = point_4f[1];
    point_out.z = point_4f[2];
    return point_out;
}
geometry_msgs::Pose transformPose(geometry_msgs::Pose pose_in, Eigen::Matrix4f transf){
    geometry_msgs::Pose pose_out;
    // Obtener rotación desde matriz de transformación
    Eigen::Matrix4f rot;
    rot = transf;
    rot.col(3) = Eigen::Vector4f(0, 0, 0, 1);
    // Crear normal desde quaternion
    tf::Quaternion tf_q;
    tf::quaternionMsgToTF(pose_in.orientation, tf_q);
    tf::Vector3 normal(1, 0, 0);
    normal = tf::quatRotate(tf_q, normal);
    normal.normalize();
    // Rotar normal
    Eigen::Vector3f normal_vector (normal.x(), normal.y(), normal.z());
    Eigen::Vector3f normal_rotated = transformVector(normal_vector, transf);
    normal_rotated.normalize();
    // Obtener quaternion desde normal rotada
    pose_out.orientation = coefsToQuaternionMsg(normal_rotated[0], normal_rotated[1], normal_rotated[2]);
    // Transportar posición
    pose_out.position = transformPoint(pose_in.position, transf);
    return pose_out;
}
geometry_msgs::Quaternion transformOrientation(geometry_msgs::Quaternion q, Eigen::Matrix4f tf){
	// Obtener componente de rotacion de la matriz
	tf::Matrix3x3 rotation (tf(0,0), tf(0,1), tf(0,2), tf(1,0), tf(1,1), tf(1,2), tf(2,0), tf(2,1), tf(2,2));
	// Convertir a quaternion tf
	tf::Quaternion rot_q;
	rotation.getRotation(rot_q);
	// Convertir quaternionmsg a tf
	tf::Quaternion in_q (q.x, q.y, q.z, q.w);
	// Añadir rotaciones
	rot_q *= in_q; // OJO: SERÁ EL ORDEN CORRECTO?
	// Crear nuevo quaternion
	geometry_msgs::Quaternion new_q;
	new_q.x = rot_q.x();
	new_q.y = rot_q.y();
	new_q.z = rot_q.z();
	new_q.w = rot_q.w();
	return new_q;
}
geometry_msgs::Pose transformPose2(geometry_msgs::Pose pose_in, Eigen::Matrix4f tf){
	geometry_msgs::Pose pose_out;
	pose_out.position = transformPoint(pose_in.position, tf);
	pose_out.orientation = transformOrientation(pose_in.orientation, tf);
	return pose_out;
}





int main(int argc, char **argv){
	ros::init(argc, argv, "transform_test");
	ros::NodeHandle nh;
	ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose1", 1);
	ros::Publisher pose_pub2 = nh.advertise<geometry_msgs::PoseStamped>("pose2", 1);
	ros::Duration(1).sleep();
	ROS_INFO("Iniciado");
	geometry_msgs::PoseStamped pose1;
	pose1.header.frame_id = "l_gripper_tool_frame";
	pose1.pose.position.x = 1;
	pose1.pose.position.y = 1;
	pose1.pose.position.z = 1;
	pose1.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,Util::PI/4);
	pose_pub.publish(pose1);
	pose_pub.publish(pose1);
	pose_pub.publish(pose1);
	Eigen::Matrix4f tf = Util::getTransformation(pose1.header.frame_id, Util::ODOM_FRAME);
	geometry_msgs::PoseStamped pose1_tf;
	pose1_tf.header.frame_id = Util::ODOM_FRAME;
	pose1_tf.pose = transformPose2(pose1.pose, tf);
	pose_pub2.publish(pose1_tf);
	pose_pub2.publish(pose1_tf);
	pose_pub2.publish(pose1_tf);
	ros::Duration(1).sleep();
	return 0;
}