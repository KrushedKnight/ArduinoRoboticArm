# Braccio Vision: AI-Powered Robotic Arm Control

A portfolio project demonstrating real-time computer vision control of an Arduino Braccio robotic arm. This system uses Python (MediaPipe) for hand tracking and C++ for high-performance Inverse Kinematics (IK) solving.

![System Architecture](https://via.placeholder.com/800x400?text=System+Architecture+ASCII+Representation)

## Field of View & Architecture

-   **Vision Layer (Python)**: Tracks hand landmarks using MediaPipe. Maps 2D screen coordinates to 3D robot space. Streams target coordinates via UDP.
-   **Control Layer (C++)**: Listens for UDP packets. Solves 6-DOF Inverse Kinematics analytically. Commands the Arduino via Serial.
-   **Firmware (Arduino)**: Executes servo commands (Braccio Shield).

## Features

-   **Analytical IK Solver**: Fast, exact geometric solution for the Braccio 6-DOF arm (vs. iterative descent).
-   **Real-time Vision**: Low-latency UDP communication between Vision and Control layers.
-   **Safety Limits**: Enforced joint constraints and workspace boundaries.
-   **Modular Design**: Decoupled Math (`IKSolver`), Hardware (`Arm`), and Vision logic.

## Getting Started

### Prerequisites

-   **Hardware**: Arduino Braccio Arm, Webcam.
-   **Software**:
    -   CMake & C++ Compiler (C++20 recommended)
    -   Python 3.x
    -   SDL2

### Installation

1.  **Build the Controller (C++)**:
    ```bash
    mkdir build && cd build
    cmake ..
    make
    ```

2.  **Setup Vision (Python)**:
    ```bash
    pip install opencv-python mediapipe numpy
    ```

### Usage

1.  **Start the Controller**:
    ```bash
    # Ensure Arduino is connected to /dev/ttyACM0
    ./ArduinoRoboticArm
    ```
    *The controller will start listening on UDP port 8080.*

2.  **Start Vision Tracking**:
    ```bash
    python3 test.py
    ```
    *A window will open showing the camera feed. Move your hand to control the robot!*

## Technical Details

### Inverse Kinematics
The solver uses a geometric approach, decomposing the arm into:
-   **Planar R-Z**: Solving the base angle and effective reach/height.
-   **3-Link Planar Chain**: Solving Shoulder, Elbow, and Wrist using the Law of Cosines.

### Communication
-   **UDP**: Protocol `x,y,z,phi` (string) for minimal latency.
-   **Serial**: 9600 baud command stream to Arduino.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
