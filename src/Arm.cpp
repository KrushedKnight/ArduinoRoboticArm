//
// Created by beast-machine-2 on 8/1/25.
//

#include "../include/Arm.h"

#include "../include/Constants.h"

Arm::Arm() {
  base = Servo{'b', false, 0, Constants::BASE_OFFSET, 0, 270};
  shoulder = Servo{'s', false, 45, Constants::SHOULDER_OFFSET, 15, 165};
  elbow = Servo{'e', false, 0, Constants::ELBOW_OFFSET, 0, 180};
  wrist_ver = Servo{'v', true, 0, Constants::WRIST_OFFSET, 0, 180};
  wrist_rot = Servo{
      'r', false, 90, 0, 0, 180,
  };
  gripper = Servo{'g', false, 10, 0, 10, 73};

  servos.push_back(&base);
  servos.push_back(&shoulder);
  servos.push_back(&elbow);
  servos.push_back(&wrist_ver);
  servos.push_back(&wrist_rot);
  servos.push_back(&gripper);
}

void Arm::apply(JointAngles angles) {
  base.position = angles.base;
  shoulder.position = angles.shoulder;
  elbow.position = angles.elbow;
  wrist_ver.position = angles.wrist_ver;
  wrist_rot.position = angles.wrist_rot;
  gripper.position = angles.gripper;
}

Arm::Arm(int basePos, int shoulderPos, int elbowPos, int wrist_verPos,
         int wrist_rotPos, int gripperPos) {
  base = Servo{'b', false, basePos, Constants::BASE_OFFSET, 0, 270};
  shoulder =
      Servo{'s', false, shoulderPos, Constants::SHOULDER_OFFSET, 15, 165};
  elbow = Servo{'e', false, elbowPos, Constants::ELBOW_OFFSET, 0, 180};
  wrist_ver = Servo{'v', true, wrist_verPos, Constants::WRIST_OFFSET, 0, 180};
  wrist_rot = Servo{
      'r', false, wrist_rotPos, 0, 0, 180,
  };
  gripper = Servo{'g', false, gripperPos, 0, 10, 73};

  servos.push_back(&base);
  servos.push_back(&shoulder);
  servos.push_back(&elbow);
  servos.push_back(&wrist_ver);
  servos.push_back(&wrist_rot);
  servos.push_back(&gripper);
}
