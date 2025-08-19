//
// Created by beast-machine-2 on 8/2/25.
//

#ifndef IKSOLVER_H
#define IKSOLVER_H
#include "Arm.h"


class IKSolver {
public:
    Arm analyticalSolve(double x, double y, double z, double phi);

};



#endif //IKSOLVER_H
