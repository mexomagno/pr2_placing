#ifndef ROBOTDRIVER_H
#define ROBOTDRIVER_H

#include <ros/ros.h>
#include "RobotHeadDriver.h"
#include "RobotBaseDriver.h"
#include "RobotGripperDriver.h"
#include "RobotSensors/RobotSensors.h"

class RobotDriver{
    public:
        // Variables públicas
        // Driver de cabeza
        RobotHeadDriver *head;
        // Driver de base
        RobotBaseDriver *base;
        // Driver de gripper izquierdo
        RobotGripperDriver *lgripper;
        // Driver de gripper derecho
        RobotGripperDriver *rgripper;
        // Sensores del robot (sólo los que ocuparé en la memoria)
        RobotSensors *sensors;
        // Constructor y destructor
        RobotDriver();
        ~RobotDriver();
    protected:
    private:
};

#endif // ROBOTDRIVER_H
