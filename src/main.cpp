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

#include "Arm.h"
#include "Constants.h"
#include "IKSolver.h"

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

double braccioToCartesian(double b) {
  return b * Constants::DEGREES_TO_RADIANS - M_PI_2;
}

double cartesianToBraccio(double a) {
  return a + M_PI_2 * Constants::RADIANS_TO_DEGREES;
}

void moveServo(Servo &servo, int amount, int serialPort) {
  servo.position += amount;
  std::string data = servo.code + std::to_string(servo.position) + "\n";
  writeCommand(serialPort, data);
}

void applyArmPosition(Arm arm, int serialPort) {
  for (Servo *servo : arm.servos) {
    std::string data =
        servo->code +
        std::to_string(cartesianToBraccio(servo->position + servo->offset)) +
        "\n";
    writeCommand(serialPort, data);
  }
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
    char buffer[1024];
    socklen_t len = sizeof(cliaddr);
    long n = recvfrom(sockfd, (char *)buffer, 1024, 0,
                      (struct sockaddr *)&cliaddr, &len);

    if (n > 0) {
      buffer[n] = '\0';
      std::string msg(buffer);
      // Parse "x,y,z,phi"
      // Simple parsing
      try {
        size_t pos1 = msg.find(',');
        size_t pos2 = msg.find(',', pos1 + 1);
        size_t pos3 = msg.find(',', pos2 + 1);

        if (pos1 != std::string::npos && pos2 != std::string::npos &&
            pos3 != std::string::npos) {
          double x = std::stod(msg.substr(0, pos1));
          double y = std::stod(msg.substr(pos1 + 1, pos2 - pos1 - 1));
          double z = std::stod(msg.substr(pos2 + 1, pos3 - pos2 - 1));
          double phi = std::stod(msg.substr(pos3 + 1));

          std::cout << "Target: " << x << ", " << y << ", " << z << "\n";

          JointAngles result = ik_solver.analyticalSolve(x, y, z, phi);
          arm.apply(result);
          applyArmPosition(arm, serialPort);
        }
      } catch (const std::exception &e) {
        std::cerr << "Parse error: " << e.what() << "\n";
      }
    }

    SDL_Delay(10); // Check every 10ms
  }

  return 0;
}
