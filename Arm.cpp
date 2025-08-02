//
// Created by beast-machine-2 on 8/1/25.
//

#include "Arm.h"


Arm::Arm() {
    Servo base       = {90, 0, 180};
    Servo shoulder   = {45,  15, 165};
    Servo elbow      = {180, 0, 180};
    Servo wrist_ver  = {180,  0, 180};
    Servo wrist_rot  = {90, 0, 180};
    Servo gripper    = {10, 10, 73};


    servos.push_back(&base);
    servos.push_back(&shoulder);
    servos.push_back(&elbow);
    servos.push_back(&wrist_ver);
    servos.push_back(&wrist_rot);
    servos.push_back(&gripper);

    codes = {'b', 's', 'e', 'v', 'r', 'g'};

}
