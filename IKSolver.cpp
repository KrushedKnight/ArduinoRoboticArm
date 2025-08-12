//
// Created by beast-machine-2 on 8/2/25.
//

#include "IKSolver.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <ostream>

#include "Constants.h"

//TODO: write transforms from our plane to actual servo angles

Arm IKSolver::analyticalSolve(double x, double y, double z, double phi) {

    if (x * x + y*y + z*z > Constants::MAX_REACH * Constants::MAX_REACH) {
        std::cerr << "The solution is too large!" << std::endl;
        std::exit(1);
    }
    z = z - Constants::BASE_HEIGHT;
    phi = phi * Constants::DEGREES_TO_RADIANS;

    double baseAngle = atan2(y,x);
    double r = sqrt(x*x + y*y);

    double P_x = r - cos(phi) * Constants::WRIST_LENGTH;
    double P_z = z - sin(phi) * Constants::WRIST_LENGTH;

    double d = sqrt(P_x*P_x + P_z*P_z);

    //TODO: check negative/positve conventions
    double cosElbow = (d * d - Constants::SHOULDER_LENGTH * Constants::SHOULDER_LENGTH - Constants::ELBOW_LENGTH * Constants::ELBOW_LENGTH)
        / (2.0 * Constants::SHOULDER_LENGTH * Constants::ELBOW_LENGTH);
    cosElbow = std::max(-1.0, std::min(1.0, cosElbow));
    double elbowAngle = acos(cosElbow);

    double angleToTarget = atan2(P_z, P_x);
    double cosShoulder = (Constants::SHOULDER_LENGTH*Constants::SHOULDER_LENGTH + d*d - Constants::ELBOW_LENGTH*Constants::ELBOW_LENGTH)
                     / (2.0 * Constants::SHOULDER_LENGTH * d);
    cosShoulder = std::max(-1.0, std::min(1.0, cosShoulder));
    double shoulderInnerAngle = acos(cosShoulder);
    double shoulderAngle = angleToTarget + shoulderInnerAngle;

    double wristAngle = phi - shoulderAngle - elbowAngle;
    //90 placeholder - make level with object

    Arm result{
        baseAngle * Constants::RADIANS_TO_DEGREES,
        shoulderAngle * Constants::RADIANS_TO_DEGREES,
        elbowAngle * Constants::RADIANS_TO_DEGREES,
        wristAngle * Constants::RADIANS_TO_DEGREES,
        0,
        Constants::GRIPPER_OPEN};
    return result;

}
