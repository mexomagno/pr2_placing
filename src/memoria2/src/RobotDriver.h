#ifndef ROBOTDRIVER_H
#define ROBOTDRIVER_H

#include "RobotHeadDriver.h"
// #include "RobotBaseDriver.h"

class RobotDriver{
    public:
        // Variables públicas
        // Driver de cabeza
        RobotHeadDriver head;
    /*    // Driver de base
        RobotBaseDriver base;
        // Driver de gripper izquierdo
        RobotGripperDriver lgripper;
        // Driver de gripper derecho
        RobotGripperDriver rgripper;*/
        RobotDriver();
    protected:
    private:
};

#endif // ROBOTDRIVER_H
