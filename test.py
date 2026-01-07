import cv2
import mediapipe as mp
import numpy as np

from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import warnings

def degrees(radians):
    return (radians / 3.14) * 180

#Todo: 2. add a2b/general conversion 3. set up ssh rq 4. quick test + debug if needed 5. add hand tracking 6. add serial communication


L0 = 0.06
L1 = 0.125
L2 = 0.125
L3 = 0.06

#TERRIBLE CODING PRACTICES!!!! WOOO
warnings.filterwarnings("ignore", message="Link .* is of type 'fixed' but set as active.*")

active_mask = [True, True, True, True, True, True, False]

arm_chain = Chain(name='arm', links=[
    OriginLink(),
    URDFLink(
        name="base",
        origin_translation=[0, 0, 0.001],
        origin_orientation=[0,0,0],
        rotation=[0, 0, 1],
        bounds=(-np.pi/3, np.pi * 1.5)
        
    ),
    URDFLink(
        name="shoulder",
        origin_translation=[0, 0, L0],
        origin_orientation=[0,0,0],
        rotation=[1, 0, 0],
        bounds=(-np.pi/2, np.pi/2)
    ),
    URDFLink(
        name="elbow",
        origin_translation=[0, 0, L1],
        origin_orientation=[0,0,0],
        rotation=[1, 0, 0],
        bounds=(-np.pi/2, np.pi/2)
    ),
    URDFLink(
        name="wrist_vert",
        origin_translation=[0, 0, L2],
        origin_orientation=[0,0,0],
        rotation=[1, 0, 0],
        bounds=(-np.pi/2, np.pi/2)
    ),
    URDFLink(
        name="wrist_rot",
        origin_translation=[0, 0, 0],
        origin_orientation=[0,0,0],
        rotation=[0, 0, 1],
    ),
    URDFLink(
        name="gripper",
        origin_translation=[0, 0, L3],
        origin_orientation=[0,0,0],
        joint_type = 'fixed'
    )
], active_links_mask = active_mask)

target = [0.1, 0.1, 0.2]  # (x, y, z) in meters
angles = arm_chain.inverse_kinematics(target)

base = 90 + degrees(angles[1])
shoulder = 90 - degrees(angles[2])
elbow = 90 + degrees(angles[3])
wrist_v = 90 - degrees(angles[4])
wrist_r = 90 + degrees(angles[5])
gripper = 10

adjustedAngles = [base, shoulder, elbow, wrist_v, wrist_r, gripper]

for x in adjustedAngles:
    print(x)

# --- Configuration ---
UDP_IP = "127.0.0.1"
UDP_PORT = 8080
ROBOT_X_MIN, ROBOT_X_MAX = -0.15, 0.15
ROBOT_Y_MIN, ROBOT_Y_MAX = 0.15, 0.35
ROBOT_Z_MIN, ROBOT_Z_MAX = 0.05, 0.30

# Initialize UDP Socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def map_value(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# --- MediaPipe Setup ---
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.7
)

# --- Camera Setup ---
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

print(f"Starting Vision System... NOTE: press 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    # Flip frame horizontally for selfie-view
    frame = cv2.flip(frame, 1)
    h, w, c = frame.shape
    
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(frame_rgb)

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp_drawing.draw_landmarks(
                frame,
                hand_landmarks,
                mp_hands.HAND_CONNECTIONS
            )

            # Use Index Finger Tip (Landmark 8) for tracking
            # Normalize coordinates [0,1]
            idx_x = hand_landmarks.landmark[8].x
            idx_y = hand_landmarks.landmark[8].y
            idx_z = hand_landmarks.landmark[8].z # Relative depth

            # --- Coordinate Mapping ---
            # Screen X (0..1) -> Robot X (Left..Right)
            # Screen Y (0..1) -> Robot Z (Up..Down) (Inverted because screen Y is down)
            # Robot Y (Reach) -> Fixed or maybe mapped to Screen Y?
            
            # Let's map:
            # Screen X -> Robot Base Rotation (X coordinate)
            # Screen Y -> Robot Height (Z coordinate)
            # Reach (Robot Y) -> Fixed for now, or ensure it's in range.
            
            target_x = map_value(idx_x, 0, 1, ROBOT_X_MIN, ROBOT_X_MAX)
            target_z = map_value(idx_y, 0, 1, ROBOT_Z_MAX, ROBOT_Z_MIN) # Inverted Y
            target_y = 0.20 # Fixed reach for now to keep it simple safe plane

            # Format: "x,y,z,phi"
            message = f"{target_x:.3f},{target_y:.3f},{target_z:.3f},0.0"
            
            try:
                sock.sendto(message.encode(), (UDP_IP, UDP_PORT))
                # Visualize on screen
                cv2.putText(frame, f"Target: {message}", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            except Exception as e:
                print(f"UDP Error: {e}")

    cv2.imshow("Robot Vision Control", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()