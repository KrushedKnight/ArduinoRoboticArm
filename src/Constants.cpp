//
// Created by beast-machine-2 on 8/2/25.
//
#include "../include/Constants.h"
#include <cmath>

namespace Constants {
const double BASE_HEIGHT = 0.06;
const double SHOULDER_LENGTH = 0.125;
const double ELBOW_LENGTH = 0.125;
const double WRIST_LENGTH = 0.06;

const double MAX_REACH = 0.370;
const double MIN_REACH = 0.1;

const double GRIPPER_OPEN = 10;
const double GRIPPER_CLOSED = 70;

const double RADIANS_TO_DEGREES = 57.2957795;
const double DEGREES_TO_RADIANS = 0.0174532925;

const int BASE_OFFSET = 65;
const int SHOULDER_OFFSET = 0;
const int ELBOW_OFFSET = 0;
const int WRIST_OFFSET = 0;

// TODO: Check all of these
const double SHOULDER_MIN = -M_PI_2;
const double SHOULDER_MAX = M_PI_2;
const double ELBOW_MIN = -M_PI;
const double ELBOW_MAX = 0;
const double WRIST_MIN = -M_PI;
const double WRIST_MAX = 0;
const double BASE_MIN = -60;
const double BASE_MAX = 220;

const double scaling_factor = 0.000075;

} // namespace Constants
