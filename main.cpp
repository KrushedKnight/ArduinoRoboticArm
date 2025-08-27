#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <SDL2/SDL.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include "Arm.h"
#include "Constants.h"
#include "IKSolver.h"
#include "Points.h";

int openSerial(const std::string& device) {
    int fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        std::cerr << "Error opening serial device\n";
        exit(1);
    }

    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Error from tcgetattr\n";
        exit(1);
    }

    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_iflag = IGNPAR;
    tty.c_oflag = 0;
    tty.c_lflag = 0;



    tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error from tcsetattr\n";
        exit(1);
    }
    int flags = TIOCM_DTR | TIOCM_RTS;
    if (ioctl(fd, TIOCMBIC, &flags) != 0) {
        std::cerr << "Warning: Could not clear DTR/RTS\n";
    }


    return fd;
}

void writeCommand(int serialPort, std::string &data) {
    int success = write(serialPort, data.c_str(), data.size());
    tcdrain(serialPort);
    std::cout << success << std::endl;
    std::cout << data << std::endl;
}




double braccioToCartesian(double b){
    return b * Constants::DEGREES_TO_RADIANS - M_PI_2;
}

double cartesianToBraccio(double a) {
    return a + M_PI_2 * Constants::RADIANS_TO_DEGREES;
}

Point3D screenToCartesian(Point2D point) {

    double x = (point.x - 320.0) * Constants::scaling_factor;
    double y = 0;
    double z = (240.0 - point.y) * Constants::scaling_factor;
    Point3D armPosition = {x, y, z};
    return armPosition;
}

void moveServo(Servo &servo, int amount, int serialPort) {
    servo.position += amount;
    std::string data = servo.code + std::to_string(servo.position) + "\n";
    writeCommand(serialPort, data);

}

void applyArmPosition(Arm arm, int serialPort) {
    for (Servo* servo : arm.servos) {
        std::string data = servo-> code + std::to_string(cartesianToBraccio(servo->position + servo->offset)) + "\n";
        writeCommand(serialPort, data);
    }
}

int main() {
    if (SDL_Init(SDL_INIT_EVENTS | SDL_INIT_VIDEO) < 0) {
        std::cerr << "Error SDL_Init\n";
    }

    SDL_Window* window = SDL_CreateWindow("Arduino Control",
                                      SDL_WINDOWPOS_CENTERED,
                                      SDL_WINDOWPOS_CENTERED,
                                      640, 480,
                                      SDL_WINDOW_SHOWN);
    if (!window) {
        std::cerr << "Error creating SDL window: " << SDL_GetError() << "\n";
        SDL_Quit();
        return 1;
    }


    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer)
    {
        std::cerr << "Failed to create renderer: " << SDL_GetError() << "\n";
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    int serialPort = openSerial("/dev/ttyACM0");
    sleep(2);


    std::string data;
    bool running = true;
    SDL_Event event;
    Arm arm = Arm();
    IKSolver ik_solver;
    std::vector<Point2D> pointsDrawn;


    while (running)
    {
        while (SDL_PollEvent(&event))
        {
            if (event.type == SDL_QUIT)
            {
                running = false;
            }

            if (event.type == SDL_MOUSEBUTTONDOWN)
            {
                if (event.button.button == SDL_BUTTON_LEFT)
                {
                    Point2D pointDrawn = {static_cast<double>(event.button.x), static_cast<double>(event.button.y)};
                    pointsDrawn.push_back(pointDrawn);
                    std::cout << "Clicked at: (" << pointDrawn.x << ", " << pointDrawn.y << ")\n";
                }
            }
        }

        // Clear screen
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255); // White background
        SDL_RenderClear(renderer);

        // Draw all points
        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255); // Red points
        for (const auto& p : pointsDrawn)
        {
            SDL_RenderDrawPoint(renderer, p.x, p.y);
        }

        // Present the drawing
        SDL_RenderPresent(renderer);
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    Point3D startingPosition = {0, 0, 0}; //fix
    Point3D relativeCoordinates;
    Point3D currentPosition;

    Arm currentArm = null;
    for (Point2D p : pointsDrawn) {
        relativeCoordinates = screenToCartesian(p);
        currentPosition = {startingPosition.x + relativeCoordinates.x,
            startingPosition.y + relativeCoordinates.y,
            startingPosition.z + relativeCoordinates.z};

        currentArm = ik_solver.analyticalSolve(currentPosition.x, currentPosition.y, currentPosition.z, 0.0);
        sleep(7.0);

    }



    return 0;
}

