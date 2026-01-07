# Braccio Vision: AI-Powered Robotic Arm Control

A portfolio project demonstrating real-time computer vision control of an Arduino Braccio robotic arm. This system uses Python (MediaPipe) for hand tracking and C++ for high-performance Inverse Kinematics (IK) solving.

![System Architecture](https://via.placeholder.com/800x400?text=System+Architecture+ASCII+Representation)


## Table of Contents
- [Motivation](#motivation)
- [Field of View & Architecture](#field-of-view--architecture)
  - [1. Vision Layer (Python)](#1-vision-layer-python)
  - [2. Control Layer (C++)](#2-control-layer-c)
  - [3. Firmware Layer (Arduino)](#3-firmware-layer-arduino)
- [Features](#features)
- [Getting Started](#getting-started)
  - [Prerequisites](#prerequisites)
  - [Installation](#installation)
  - [Usage](#usage)
- [Technical Details](#technical-details)
  - [Inverse Kinematics](#inverse-kinematics)
  - [Communication](#communication)
- [License](#license)

## Motivation

Industrial and surgical robotic systems prioritize **intuitive human–machine interfaces** over direct joint-level control. Inspired by the design philosophy behind **Intuitive Surgical’s da Vinci system**, this project explores how natural hand gestures can be mapped to precise robotic motion while preserving safety and determinism.

Rather than controlling individual joints, the user operates the arm through **relative end-effector motion**, allowing them to focus on intent instead of kinematics. A virtual clutch mechanism enables repositioning without unintended robot movement—mirroring interaction patterns used in professional teleoperation systems.

## Field of View & Architecture

### 1. Vision Layer (Python)
- Tracks 21 hand landmarks using **MediaPipe**
- Converts 2D screen coordinates into a normalized 3D workspace
- Streams target pose data via **binary UDP packets**

### 2. Control Layer (C++)
- Receives UDP packets and validates inputs
- Solves **6-DOF inverse kinematics analytically**
- Enforces joint limits and workspace constraints
- Streams servo commands to the Arduino using a compact binary protocol

### 3. Firmware Layer (Arduino)
- Runs on the Braccio Shield
- Parses high-speed binary frames
- Updates servo positions deterministically

## Features

- **Analytical Inverse Kinematics**
  - Closed-form geometric solution (no numerical solvers)
  - Predictable runtime and deterministic behavior

- **Gesture-Based Control**
  - **Relative Positioning**: Hand motion maps directly to end-effector motion
  - **Virtual Clutch**: Make a fist to pause tracking and reposition your hand
  - **Wrist Pitch Mimicry**: End-effector pitch follows hand orientation

- **Low-Latency Communication**
  - Binary UDP protocol (~80% smaller than text-based messages)
  - High-speed serial communication (115200 baud)

- **Safety Constraints**
  - Joint angle limits enforced in software
  - Workspace clamping to prevent self-collision or overextension

---

## Getting Started

### Prerequisites

-   **Hardware**: 
    - Arduino Braccio Arm
    - Webcam
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

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
