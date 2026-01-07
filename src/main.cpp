#include <SDL2/SDL.h>
#include <arpa/inet.h>
#include <cmath>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <netinet/in.h>
#include <sstream>
#include <string>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include "../include/Arm.h"
#include "../include/Constants.h"
#include "../include/IKSolver.h"

int openSerial(const std::string &device) {
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

  cfsetospeed(&tty, B115200);
  cfsetispeed(&tty, B115200);

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

// writeCommand removed (Binary Protocol used)
double braccioToCartesian(double b) {
  return b * Constants::DEGREES_TO_RADIANS - M_PI_2;
}

double cartesianToBraccio(double a) {
  return a + M_PI_2 * Constants::RADIANS_TO_DEGREES;
}

void moveServo(Servo &servo, int amount, int serialPort) {
  servo.position += amount;
  // NOTE: Single servo move not fully supported in simple binary protocol
  // without sending full frame. For key-press debug, we should really resend
  // full arm. Ignoring for now to focus on IK loop.
}

// Binary Frame: [0xFF, Base, Shoulder, Elbow, WristV, WristR, Gripper]
void applyArmPosition(Arm arm, int serialPort) {
  uint8_t frame[7];
  frame[0] = 0xFF; // Sync Byte

  int i = 1;
  for (Servo *servo : arm.servos) {
    // Map 0-180 degrees to byte.
    // Clamp to ensure we don't accidentally send 0xFF (Sync) as data if
    // possible, though strictly 0xFF is valid data 255, but servo range is
    // 0-180. Braccio servos are 0-180.
    int angle = (int)cartesianToBraccio(servo->position + servo->offset);
    if (angle < 0)
      angle = 0;
    if (angle > 180)
      angle = 180;

    frame[i] = (uint8_t)angle;
    i++;
  }

  write(serialPort, frame, 7);
  tcdrain(serialPort);
}
int main() {
  if (SDL_Init(SDL_INIT_EVENTS | SDL_INIT_VIDEO) < 0) {
    std::cerr << "Error SDL_Init\n";
  }

  SDL_Window *window =
      SDL_CreateWindow("Arduino Control", SDL_WINDOWPOS_CENTERED,
                       SDL_WINDOWPOS_CENTERED, 640, 480, SDL_WINDOW_SHOWN);
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
  IKSolver ik_solver;

  // --- UDP Setup ---
  int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0) {
    std::cerr << "Error creating socket\n";
    return 1;
  }

  struct sockaddr_in servaddr {};
  struct sockaddr_in cliaddr {};
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = INADDR_ANY;
  servaddr.sin_port = htons(8080);

  if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
    std::cerr << "Bind failed\n";
    return 1;
  }

  // Set non-blocking
  int flags = fcntl(sockfd, F_GETFL, 0);
  fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);

  std::cout << "Listening for UDP on 8080...\n";

  while (running) {
    while (SDL_PollEvent(&event)) {
      if (event.type == SDL_QUIT) {
        running = false;
      } else if (event.type == SDL_KEYDOWN) {
        if (event.key.keysym.sym == SDLK_ESCAPE)
          running = false;
      }
    }

    // --- UDP Poll ---
    // Packet Structure: [x, y, z, phi] as 4 floats (16 bytes)
    char buffer[16];
    socklen_t len = sizeof(cliaddr);
    long n = recvfrom(sockfd, (char *)buffer, 16, 0,
                      (struct sockaddr *)&cliaddr, &len);

    if (n == 16) {
      // Direct cast for speed (assuming Little Endian on both sides - typical
      // for x86/ARM)
      float *data = (float *)buffer;
      double x = (double)data[0];
      double y = (double)data[1];
      double z = (double)data[2];
      double phi = (double)data[3];

      std::cout << "Target: " << x << ", " << y << ", " << z << "\n";

      JointAngles result = ik_solver.analyticalSolve(x, y, z, phi);
      arm.apply(result);
      applyArmPosition(arm, serialPort);
    }

    SDL_Delay(10); // Check every 10ms
  }

  return 0;
}
