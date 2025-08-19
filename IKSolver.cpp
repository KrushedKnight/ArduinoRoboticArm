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

        z = z - Constants::BASE_HEIGHT;

        double totalDistance = sqrt(x*x + y*y + z*z);
        if (totalDistance > Constants::MAX_REACH) {
            std::cerr << "The solution is too large!" << std::endl;
            std::exit(1);
        }

        if (totalDistance < Constants::MIN_REACH) {
            std::cerr << "Target is too close to base! Distance: " << totalDistance;
            std::exit(1);
        }

        phi = phi * Constants::DEGREES_TO_RADIANS;

        double baseAngle = atan2(y,x);
        double r = sqrt(x*x + y*y);

        double P_x = r - cos(phi) * Constants::WRIST_LENGTH;
        double P_z = z - sin(phi) * Constants::WRIST_LENGTH;

        double d = sqrt(P_x*P_x + P_z*P_z);

        //TODO: check negative/positve conventions
        double cosElbow = (Constants::SHOULDER_LENGTH * Constants::SHOULDER_LENGTH + Constants::ELBOW_LENGTH * Constants::ELBOW_LENGTH - d * d)
            / (2.0 * Constants::SHOULDER_LENGTH * Constants::ELBOW_LENGTH);
        cosElbow = std::max(-1.0, std::min(1.0, cosElbow));
        double elbowAngleUp = acos(cosElbow);
        double elbowAngleDown = -acos(cosElbow);



        double angleToTarget = atan2(P_z, P_x);
        double cosShoulder = (Constants::SHOULDER_LENGTH*Constants::SHOULDER_LENGTH + d * d - Constants::ELBOW_LENGTH*Constants::ELBOW_LENGTH)
                         / (2.0 * Constants::SHOULDER_LENGTH * d);
        cosShoulder = std::max(-1.0, std::min(1.0, cosShoulder));
        double shoulderInnerAngle = acos(cosShoulder);

        double shoulderAngleUp = angleToTarget - shoulderInnerAngle;
        double shoulderAngleDown = angleToTarget + shoulderInnerAngle;

        double wristAngleUp = phi - shoulderAngleUp - elbowAngleUp;
        double wristAngleDown = phi - shoulderAngleDown - elbowAngleDown;

        double shoulderAngle, elbowAngle, wristAngle;

        bool upValid = (shoulderAngleUp >= Constants::SHOULDER_MIN &&
                    shoulderAngleUp <= Constants::SHOULDER_MAX &&
                    elbowAngleUp >= Constants::ELBOW_MIN &&
                    elbowAngleUp <= Constants::ELBOW_MAX);

        bool downValid = (shoulderAngleDown >= Constants::SHOULDER_MIN &&
                          shoulderAngleDown <= Constants::SHOULDER_MAX &&
                          elbowAngleDown >= Constants::ELBOW_MIN &&
                          elbowAngleDown <= Constants::ELBOW_MAX);

        if (upValid) {
            shoulderAngle = shoulderAngleUp;
            elbowAngle = elbowAngleUp;
            wristAngle = wristAngleUp;
        } else if (downValid) {
            shoulderAngle = shoulderAngleDown;
            elbowAngle = elbowAngleDown;
            wristAngle = wristAngleDown;
        }
        else {
            std::cerr<< "No valid position found";
            std::exit(1);
        }


        std::cout << "=== IK Debug ===" << std::endl;
        std::cout << "Target: (" << x << ", " << y << ", " << z << "), phi: " << phi * 180/M_PI << "°" << std::endl;
        std::cout << "P_x: " << P_x << ", P_z: " << P_z << ", d: " << d << std::endl;
        std::cout << "cosElbow: " << cosElbow << std::endl;
        std::cout << "Raw acos(cosElbow): " << acos(cosElbow) * 180/M_PI << "°" << std::endl;
        std::cout << "angleToTarget: " << angleToTarget * 180/M_PI << "°" << std::endl;
        std::cout << "shoulderInnerAngle: " << shoulderInnerAngle * 180/M_PI << "°" << std::endl;
        std::cout << "Final angles - Shoulder: " << shoulderAngle * 180/M_PI << "°, Elbow: " << elbowAngle * 180/M_PI << "°, Wrist: " << wristAngle * 180/M_PI << "°" << std::endl;
        std::cout << "Final angles DOWN - Shoulder: " << shoulderAngleDown * 180/M_PI << "°, Elbow: " << elbowAngleDown * 180/M_PI << "°, Wrist: " << wristAngleDown * 180/M_PI << "°" << std::endl;
        std::cout << "Final angles UP - Shoulder: " << shoulderAngleUp * 180/M_PI << "°, Elbow: " << elbowAngleUp * 180/M_PI << "°, Wrist: " << wristAngleUp * 180/M_PI << "°" << std::endl;


        Arm result{
            baseAngle * Constants::RADIANS_TO_DEGREES,
            shoulderAngle * Constants::RADIANS_TO_DEGREES,
            elbowAngle * Constants::RADIANS_TO_DEGREES,
            wristAngle * Constants::RADIANS_TO_DEGREES,
            0,
            Constants::GRIPPER_OPEN};
        return result;

    }
