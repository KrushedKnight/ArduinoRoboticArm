//
// Created by beast-machine-2 on 8/1/25.
//

#include "Arm.h"

#include "Constants.h"


Arm::Arm() {
    base       = {'b',90, Constants::BASE_OFFSET, 0, 270};
    shoulder   = {'s',45,  Constants::SHOULDER_OFFSET, 15, 165};
    elbow      = {'e',180, Constants::ELBOW_OFFSET, 0, 180};
    wrist_ver  = {'v',180,  Constants::WRIST_OFFSET, 0, 180};
    wrist_rot  = {'r',90, 0, 0, 180, };
    gripper    = {'g',10, 0, 10, 73};


    servos.push_back(&base);
    servos.push_back(&shoulder);
    servos.push_back(&elbow);
    servos.push_back(&wrist_ver);
    servos.push_back(&wrist_rot);
    servos.push_back(&gripper);


}

Arm::Arm(int basePos, int shoulderPos, int elbowPos, int wrist_verPos, int wrist_rotPos, int gripperPos) {
    base       = {'b',basePos, Constants::BASE_OFFSET, 0, 270};
    shoulder   = {'s',shoulderPos,  Constants::SHOULDER_OFFSET, 15, 165};
    elbow      = {'e',elbowPos, Constants::ELBOW_OFFSET, 0, 180};
    wrist_ver  = {'v',wrist_verPos,  Constants::WRIST_OFFSET, 0, 180};
    wrist_rot  = {'r',wrist_rotPos, 0, 0, 180, };
    gripper    = {'g',gripperPos, 0, 10, 73};


    servos.push_back(&base);
    servos.push_back(&shoulder);
    servos.push_back(&elbow);
    servos.push_back(&wrist_ver);
    servos.push_back(&wrist_rot);
    servos.push_back(&gripper);

}
