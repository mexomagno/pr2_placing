#ifndef UTIL_H_H
#define UTIL_H_H

#include <string>

using namespace std;

class Util{
    public:
        Util();
        // CONSTANTES PARA TODO EL QUE IMPORTE ESTA CLASE
        // Frames
        static const string BASE_FRAME;
        static const string CAMERA_FRAME;
        static const string KINECT_FRAME;

        // Tópicos
        static const string KINECT_TOPIC;
        static const string GRIPPER_GOAL_TOPIC_SUFFIX;
        static const string GRIPPER_STATUS_TOPIC_SUFFIX;

        // Otros
        static const float PI;
    protected:
    private:
};

#endif // UTIL_H_H
