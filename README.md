# Braccio Vision: AI-Powered Robotic Arm Control

A portfolio project demonstrating real-time computer vision control of an Arduino Braccio robotic arm. This system uses Python (MediaPipe) for hand tracking and C++ for high-performance Inverse Kinematics (IK) solving.

![System Architecture](https://via.placeholder.com/800x400?text=System+Architecture+ASCII+Representation)

## Inspiration
This project draws heavily from the design philosophy of **Intuitive Surgical** (creators of the da Vinci Surgical System). The goal was to abstract the complex kinematics of a 6-DOF robot behind a natural, gesture-based interface.

Traditional robot control often involves tedious joint-by-joint manipulation. By implementing a **"Virtual Clutch"** and relative positioning system, this project achieves a level of fluidity where the user forgets they are controlling a robot and simply moves their hand—much like a surgeon operating a remote manipulator.

## Field of View & Architecture

-   **Vision Layer (Python)**: Tracks hand landmarks using MediaPipe. Maps 2D screen coordinates to 3D robot space. Streams target coordinates via UDP.
-   **Control Layer (C++)**: Listens for UDP packets. Solves 6-DOF Inverse Kinematics analytically. Commands the Arduino via Serial.
-   **Firmware (Arduino)**: Executes servo commands (Braccio Shield).

## Features

-   **Analytical IK Solver**: Fast, exact geometric solution for the Braccio 6-DOF arm.
-   **Advanced Gesture Control**:
    -   **Virtual Mouse**: Move your hand to move the robot (Relative Positioning).
    -   **Clutch Mechanism**: Make a **FIST** to pause tracking and reposition your hand (like lifting a mouse).
    -   **Pitch Mimicry**: The robot wrist tilts up/down to match your hand's orientation.
-   **Real-time Vision**: Low-latency UDP communication between Vision and Control layers.
-   **Safe Limits**: Enforced joint constraints and workspace boundaries.
-   **Binary Protocol**: Optimized "Bitmapping" protocol reduces bandwidth by ~80% and latency.

## Getting Started

### Prerequisites

-   **Hardware**: Arduino Braccio Arm, Webcam.
-   **Software**:
    -   CMake & C++ Compiler (C++20 recommended)
    -   Python 3.x
    -   MediaPipe
    -   OpenCV
    -   NumPy

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

3.  **Flash Firmware (Arduino)**:
    -   Open `firmware/BraccioBinary.ino` in Arduino IDE.
    -   Upload to your Arduino (this enables the high-speed binary protocol).

### Usage

1.  **Start the Controller**:
    ```bash
    # Ensure Arduino is connected to /dev/ttyACM0
    ./ArduinoRoboticArm
    ```
    *The controller will start listening on UDP port 8080.*

2.  **Start Vision Tracking**:
    ```bash
    python3 scripts/vision_controller.py
    ```

3.  **Controls**:
    -   **Open Hand**: Move the robot.
    -   **Fist**: **Clutch (Pause)**. Use this to reposition your hand.
    -   **Tilt Hand**: Control Wrist Pitch.

## Technical Details

### Inverse Kinematics
The solver uses a geometric approach, decomposing the arm into:
-   **Planar R-Z**: Solving the base angle and effective reach/height.
-   **3-Link Planar Chain**: Solving Shoulder, Elbow, and Wrist using the Law of Cosines.

### Communication
-   **UDP (Vision -> Control)**: 16-byte Binary Packet (`[float x, float y, float z, float phi]`).
-   **Serial (Control -> Arduino)**: 7-byte Binary Frame (`[0xFF, Base, Shoulder, Elbow, WristV, WristR, Gripper]`).
-   **Baud Rate**: 115200 bps (Up from 9600).

## License
## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
