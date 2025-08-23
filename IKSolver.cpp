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

    bool IKSolver::cosrule(double opposite, double adjacent1, double adjacent2, double& angle) {
        double delta = 2 * adjacent1 * adjacent2;

        if (delta == 0) return false;

        double cos = (adjacent1*adjacent1 + adjacent2*adjacent2 - opposite*opposite) / delta;

        if ((cos > 1) || (cos < -1)) return false;

        angle = acos(cos);

        return true;
    }
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

        double d = sqrt(r*r + z*z);
        double theta = atan2(y,x);
        double P_x = d * cos(theta - M_PI_2);
        double P_y = d * sin(theta - M_PI_2);
        double P_phi = phi - M_PI_2;

        double xw = P_x - Constants::WRIST_LENGTH * cos(P_phi);
        double yw = P_y - Constants::WRIST_LENGTH * sin(P_phi);

        //convert to polar
        double alpha = atan2(yw, xw);
        double R = sqrt(xw*xw + yw*yw);

        double beta;
        if (!cosrule(Constants::ELBOW_LENGTH, R, Constants::SHOULDER_LENGTH, beta)) {
            std::cerr << "cos rule error!";
            std::exit(1);
        }

        double gamma;
        if (!cosrule(R, Constants::SHOULDER_LENGTH, Constants::ELBOW_LENGTH, gamma)) {
            std::cerr << "cos rule error!";
            std::exit(1);
        }

        double shoulderAngleDown = alpha - beta;
        double shoulderAngleUp = alpha + beta;

        double elbowAngleDown = M_PI - gamma;
        double elbowAngleUp = gamma - M_PI;

        double wristAngleDown = P_phi - shoulderAngleDown - elbowAngleDown;
        double wristAngleUp = P_phi - shoulderAngleUp - elbowAngleUp;


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
            // std::exit(1);
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


        std::cout << "\n=== IK Debug ===" << std::endl;

        std::cout << "Target: (" << x << ", " << y << ", " << z
                  << "), phi: " << phi * 180.0 / M_PI << "°" << std::endl;

        std::cout << "Base angle: " << baseAngle * 180.0 / M_PI << "°" << std::endl;
        std::cout << "Total distance: " << totalDistance << std::endl;
        std::cout << "Planar distance r: " << r << ", d: " << d << std::endl;

        std::cout << "Wrist reference point -> P_x: " << P_x
                  << ", P_y: " << P_y
                  << ", P_phi: " << P_phi * 180.0 / M_PI << "°" << std::endl;
        std::cout << "Wrist position (xw, yw): (" << xw << ", " << yw
                  << "), Polar R: " << R
                  << ", alpha: " << alpha * 180.0 / M_PI << "°" << std::endl;

        std::cout << "Shoulder inner angle (beta): " << beta * 180.0 / M_PI << "°" << std::endl;
        std::cout << "Elbow inner angle (gamma): " << gamma * 180.0 / M_PI << "°" << std::endl;

        std::cout << "DOWN solution -> Shoulder: " << shoulderAngleDown * 180.0 / M_PI
                  << "°, Elbow: " << elbowAngleDown * 180.0 / M_PI
                  << "°, Wrist: " << wristAngleDown * 180.0 / M_PI << "°" << std::endl;

        std::cout << "UP solution   -> Shoulder: " << shoulderAngleUp * 180.0 / M_PI
                  << "°, Elbow: " << elbowAngleUp * 180.0 / M_PI
                  << "°, Wrist: " << wristAngleUp * 180.0 / M_PI << "°" << std::endl;

        std::cout << "Chosen solution -> Shoulder: " << shoulderAngle * 180.0 / M_PI
                  << "°, Elbow: " << elbowAngle * 180.0 / M_PI
                  << "°, Wrist: " << wristAngle * 180.0 / M_PI << "°" << std::endl;

        std::cout << "FK check: (" << verify_x << ", " << verify_y << ", " << verify_z << ")" << std::endl;
        std::cout << "Position error: "
                  << sqrt(pow(x - verify_x, 2) + pow(y - verify_y, 2) + pow(z + Constants::BASE_HEIGHT - verify_z, 2))
                  << std::endl;

        Arm result{
            baseAngle * Constants::RADIANS_TO_DEGREES,
            shoulderAngle * Constants::RADIANS_TO_DEGREES,
            elbowAngle * Constants::RADIANS_TO_DEGREES,
            wristAngle * Constants::RADIANS_TO_DEGREES,
            0,
            Constants::GRIPPER_OPEN};
        return result;

    }
