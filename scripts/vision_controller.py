import cv2
import mediapipe as mp
import numpy as np

import socket
import time
import math
import struct


# --- Configuration ---
UDP_IP = "127.0.0.1"
UDP_PORT = 8080

# Robot Workspace Limits
ROBOT_X_MIN, ROBOT_X_MAX = -0.15, 0.15
ROBOT_Y_MIN, ROBOT_Y_MAX = 0.15, 0.40  # Reach (Forward/Back)
ROBOT_Z_MIN, ROBOT_Z_MAX = 0.05, 0.35  # Height

# Control Sensitivity
SENSITIVITY_X = 0.7
SENSITIVITY_Y = 0.7
SENSITIVITY_Z = 0.7

class RobotState:
    def __init__(self):
        # Start at a safe home position
        self.x = 0.0
        self.y = 0.25
        self.z = 0.20
        self.phi = 0.0  # Wrist pitch (radians)

    def update(self, dx, dy, dz):
        self.x = np.clip(self.x + dx, ROBOT_X_MIN, ROBOT_X_MAX)
        self.y = np.clip(self.y + dy, ROBOT_Y_MIN, ROBOT_Y_MAX)
        self.z = np.clip(self.z + dz, ROBOT_Z_MIN, ROBOT_Z_MAX)

    def set_pitch(self, angle_rad):
        # Clip pitch to reasonable values (-90 to 90 degrees)
        self.phi = np.clip(angle_rad, -1.57, 1.57)

    def __str__(self):
        return f"{self.x:.3f},{self.y:.3f},{self.z:.3f},{self.phi:.3f}"

# --- Helper Functions ---
def calculate_distance(p1, p2):
    return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)

def is_fist(landmarks):
    # Simple heuristic: Check if fingertips are below finger IP joints (folded)
    # 0 = Wrist
    # Tips: 8, 12, 16, 20
    # PIPs: 6, 10, 14, 18
    # Pseudo-fist: Tips are close to Palm (0) or below PIPs
    
    # Check Index, Middle, Ring, Pinky
    fingers_folded = 0
    for tip_idx, pip_idx in [(8, 6), (12, 10), (16, 14), (20, 18)]:
        # Note: Y increases downwards in screen coordinates
        if landmarks[tip_idx].y > landmarks[pip_idx].y: 
            fingers_folded += 1
            
    return fingers_folded >= 3 # If 3 or more fingers are folded

def get_wrist_pitch(landmarks):
    # Vector from Wrist(0) to Middle MPC(9)
    # We want the angle relative to "flat"
    wrist = landmarks[0]
    middle_base = landmarks[9]
    
    delta_y = wrist.y - middle_base.y # Up is negative Y in screen
    delta_z = middle_base.z - wrist.z # Forward is negative Z usually
    
    # Start simple: Map Y-difference to pitch
    # Hand pointing UP (wrist below fingers) -> Positive Pitch
    # Hand pointing DOWN (wrist above fingers) -> Negative Pitch
    
    angle = math.atan2(delta_y, 0.1) # Approx normalized run
    # Amplify for better control
    return angle * 2.0

# --- Setup ---
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.7
)

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

robot = RobotState()
last_hand_pos = None # (x, y, z)

print("--- Gesture Control Active ---")
print(" [OPEN HAND] : Move Robot")
print(" [FIST]      : Clutch (Reposition Hand)")
print(" [TILT]      : Control Wrist Pitch")

while True:
    ret, frame = cap.read()
    if not ret: break

    frame = cv2.flip(frame, 1) # Mirror view
    h, w, c = frame.shape
    results = hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

    status_text = "Searching..."
    color = (0, 0, 255)

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            
            # 1. Coordinate Extraction (Normalized 0..1)
            # Use Wrist (0) or Index MCP (5) as stable anchor for movement
            anchor = hand_landmarks.landmark[9] # Middle finger MCP is very stable center
            
            curr_x = anchor.x
            curr_y = anchor.y
            curr_z = anchor.z # Relative depth
            
            # 2. Gesture Recognition
            clutched = is_fist(hand_landmarks.landmark)
            
            # 3. Control Logic
            if last_hand_pos is None:
                last_hand_pos = (curr_x, curr_y, curr_z)
            
            if clutched:
                status_text = "CLUTCH (Paused)"
                color = (0, 255, 255) # Yellow
                # Update last pos so we don't jump when we unclutch
                last_hand_pos = (curr_x, curr_y, curr_z)
            else:
                status_text = "ACTIVE"
                color = (0, 255, 0) # Green
                
                # Calculate Deltas
                dx = (curr_x - last_hand_pos[0]) * SENSITIVITY_X
                dy = (curr_y - last_hand_pos[1]) * SENSITIVITY_Z # Screen Y -> Robot Z (Height)
                dz = (last_hand_pos[2] - curr_z) * SENSITIVITY_Y # Depth -> Robot Y (Reach). Note sign flip?
                
                # Invert Screen Y because Screen Y is Down, Robot Z is Up
                dy = -dy 
                
                # Update Robot
                robot.update(dx, dz, dy) # Mapping: ScreenX->RobX, ScreenDepth->RobY, ScreenY->RobZ
                
                # Pitch Control
                pitch = get_wrist_pitch(hand_landmarks.landmark)
                robot.set_pitch(pitch)
                
                last_hand_pos = (curr_x, curr_y, curr_z)

import struct

# ... (Previous imports)

# ... (Inside loop)

            # 4. Transmit
            # UDP Packet Structure: [float x, float y, float z, float phi] -> 16 bytes
            # Little Endian (<)
            packed_data = struct.pack('<ffff', robot.x, robot.y, robot.z, robot.phi)
            
            try:
                sock.sendto(packed_data, (UDP_IP, UDP_PORT))
            except: pass
            
            # UI
            cv2.putText(frame, f"Robot: {robot}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)


    cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
    cv2.imshow("Virtual Mouse Robot Control", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()