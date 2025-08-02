//
// Created by beast-machine-2 on 8/1/25.
//

#include "Arm.h"


Arm::Arm() {
    base       = {'b',90, 0, 270};
    shoulder   = {'s',45,  15, 165};
    elbow      = {'e',180, 0, 180};
    wrist_ver  = {'v',180,  0, 180};
    wrist_rot  = {'r',90, 0, 180};
    gripper    = {'g',10, 10, 73};


    servos.push_back(&base);
    servos.push_back(&shoulder);
    servos.push_back(&elbow);
    servos.push_back(&wrist_ver);
    servos.push_back(&wrist_rot);
    servos.push_back(&gripper);

    codes = {'b', 's', 'e', 'v', 'r', 'g'};

}
