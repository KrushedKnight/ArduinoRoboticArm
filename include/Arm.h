//
// Created by beast-machine-2 on 8/1/25.
//

#ifndef ARM_H
#define ARM_H
#include <vector>

#include "Constants.h"
#include "Servo.h"

class Arm {
public:
  Servo base;
  Servo shoulder;
  Servo elbow;
  Servo wrist_ver;
  Servo wrist_rot;
  Servo gripper;

  std::vector<Servo *> servos;

  Arm();
  Arm(int base, int shoulder, int elbow, int wrist_ver, int wrist_rot,
      int gripper);

  void apply(struct JointAngles angles);
};

#endif // ARM_H
