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

        if (baseAngle < Constants::BASE_MIN || baseAngle > Constants::BASE_MAX) {
            baseAngle += (baseAngle < 0) ? M_PI : -M_PI;
            r *= -1;
            phi = M_PI - phi;
        }



        double P_x = r;
        double P_z = z;

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
                    elbowAngleUp <= Constants::ELBOW_MAX &&
                    wristAngleUp >= Constants::WRIST_MIN &&
                    wristAngleUp <= Constants::WRIST_MAX);

        bool downValid = (shoulderAngleDown >= Constants::SHOULDER_MIN &&
                          shoulderAngleDown <= Constants::SHOULDER_MAX &&
                          elbowAngleDown >= Constants::ELBOW_MIN &&
                          elbowAngleDown <= Constants::ELBOW_MAX &&
                          wristAngleDown >= Constants::WRIST_MIN &&
                          wristAngleDown <= Constants::WRIST_MAX);

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

        double verify_x = cos(baseAngle) * (Constants::SHOULDER_LENGTH * cos(shoulderAngle) +
                                   Constants::ELBOW_LENGTH * cos(shoulderAngle + elbowAngle) +
                                   Constants::WRIST_LENGTH * cos(shoulderAngle + elbowAngle + wristAngle));
        double verify_y = sin(baseAngle) * (Constants::SHOULDER_LENGTH * cos(shoulderAngle) +
                                           Constants::ELBOW_LENGTH * cos(shoulderAngle + elbowAngle) +
                                           Constants::WRIST_LENGTH * cos(shoulderAngle + elbowAngle + wristAngle));
        double verify_z = Constants::BASE_HEIGHT + Constants::SHOULDER_LENGTH * sin(shoulderAngle) +
                         Constants::ELBOW_LENGTH * sin(shoulderAngle + elbowAngle) +
                         Constants::WRIST_LENGTH * sin(shoulderAngle + elbowAngle + wristAngle);

        std::cout << "Forward kinematics check: (" << verify_x << ", " << verify_y << ", " << verify_z << ")" << std::endl;
        std::cout << "Position error: " << sqrt(pow(x-verify_x,2) + pow(y-verify_y,2) + pow(z+Constants::BASE_HEIGHT-verify_z,2)) << std::endl;

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
