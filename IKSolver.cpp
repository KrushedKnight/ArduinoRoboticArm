//
// Created by beast-machine-2 on 8/2/25.
//

#include "IKSolver.h"

#include <cmath>

#include "Constants.h"

//TODO: write transforms from our plane to actual servo angles

Arm IKSolver::analyticalSolve(double x, double y, double z, double phi) {

    double baseAngle = atan(y/x);
    double r = sqrt(x*x + y*y);

    double P_x = r - cos(phi) * Constants::WRIST_LENGTH;
    double P_z = z - sin(phi) * Constants::WRIST_LENGTH;

    double d = sqrt(P_x*P_x + P_z*P_z);

    //TODO: check negative/positve conventions
    double elbowAngle = acos((d * d - Constants::SHOULDER_LENGTH * Constants::SHOULDER_LENGTH - Constants::ELBOW_LENGTH * Constants::ELBOW_LENGTH) / 2 * Constants::SHOULDER_LENGTH * Constants::ELBOW_LENGTH);
    double shoulderAngle = acos((Constants::SHOULDER_LENGTH * Constants::SHOULDER_LENGTH + d * d - Constants::ELBOW_LENGTH * Constants::ELBOW_LENGTH) / 2 * Constants::SHOULDER_LENGTH * Constants::ELBOW_LENGTH);

    //90 placeholder - make level with object
    Arm result{baseAngle, shoulderAngle, elbowAngle, phi, 90, Constants::GRIPPER_OPEN};
    return result;

}
