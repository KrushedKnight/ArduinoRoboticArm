//
// Created by beast-machine-2 on 8/2/25.
//

#include "IKSolver.h"

#include <cmath>

Arm IKSolver::analyticalSolve(double x, double y, double z, double phi) {
    Arm result;
    double baseAngle = atan(y/x);
    double r = sqrt(x*x + y*y);
}
