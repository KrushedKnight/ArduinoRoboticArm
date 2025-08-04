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

void moveServo(Servo &servo, int amount, int serialPort) {
    servo.position += amount;
    std::string data = servo.code + std::to_string(servo.position) + "\n";
    writeCommand(serialPort, data);

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


    int serialPort = openSerial("/dev/ttyACM0");
    sleep(2);


    std::string data;
    bool running = true;
    SDL_Event event;
    Arm arm = Arm();


    while (running) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = false;
            }
            else if (event.type == SDL_KEYDOWN) {
                if (event.key.keysym.sym == SDLK_w) {
                    moveServo(arm.shoulder, 3, serialPort);

                }
                if (event.key.keysym.sym == SDLK_s) {
                    moveServo(arm.shoulder, -3, serialPort);
                }
                if (event.key.keysym.sym == SDLK_d) {
                    moveServo(arm.base, 1, serialPort);
                }
                if (event.key.keysym.sym == SDLK_a) {
                    moveServo(arm.base, -1, serialPort);
                }
                if (event.key.keysym.sym == SDLK_t) {
                    moveServo(arm.elbow, 1, serialPort);
                }
                if (event.key.keysym.sym == SDLK_g) {
                    moveServo(arm.elbow, -1, serialPort);
                }
                if (event.key.keysym.sym == SDLK_UP) {
                    moveServo(arm.wrist_ver, 1, serialPort);
                }
                if (event.key.keysym.sym == SDLK_DOWN) {
                    moveServo(arm.wrist_ver, -1, serialPort);
                }
                if (event.key.keysym.sym == SDLK_RIGHT) {
                    moveServo(arm.wrist_rot, 1, serialPort);
                }
                if (event.key.keysym.sym == SDLK_LEFT) {
                    moveServo(arm.wrist_rot, -1, serialPort);
                }
                if (event.key.keysym.sym == SDLK_e) {
                    moveServo(arm.gripper, 1, serialPort);
                }
                if (event.key.keysym.sym == SDLK_q) {
                    moveServo(arm.gripper, -1, serialPort);
                }

                else if (event.key.keysym.sym == SDLK_ESCAPE) {
                    running = false;
                }
            }
        }
        SDL_Delay(100);
    }

    return 0;
}