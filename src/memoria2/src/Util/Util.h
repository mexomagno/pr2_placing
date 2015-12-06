#ifndef UTIL_H
#define UTIL_H

#include <string>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_listener.h>

using namespace std;

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
        static const string GRIPPER_GOAL_TOPIC_SUFFIX;
        static const string GRIPPER_STATUS_TOPIC_SUFFIX;
        static const string BASE_CONTROLLER_TOPIC;
        static const string ODOM_TOPIC;

        // Otros
        static const float PI;

        // Métodos
        static float toRad(float grad);
        static float toGrad(float rad);
        static geometry_msgs::Quaternion coefsToQuaternionMsg(float a, float b, float c);
        // static scanGripper(char which);
    protected:
    private:
};

#endif // UTIL_H
