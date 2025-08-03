//
// Created by beast-machine-2 on 8/1/25.
//

#ifndef ARM_H
#define ARM_H
#include <vector>

#include "Servo.h"


class Arm {
public:
    Servo base;
    Servo shoulder;
    Servo elbow;
    Servo wrist_ver;
    Servo wrist_rot;
    Servo gripper;

    std::vector<Servo*> servos;

    Arm();
    Arm(double base, double shoulder, double elbow, double wrist_ver, double wrist_rot, double gripper);
};



#endif //ARM_H
