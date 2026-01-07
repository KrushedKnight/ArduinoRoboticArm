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

# # mediapipe hands stuff
# mp_hands = mp.solutions.hands
# mp_drawing = mp.solutions.drawing_utils
# hands = mp_hands.Hands(
#     static_image_mode=False,
#     max_num_hands=1,
#     min_detection_confidence=0.5,
#     min_tracking_confidence=0.5
# )

# # open cv stuff
# cap = cv2.VideoCapture(0)  

# while True:
#     ret, frame = cap.read()
#     if not ret:
#         print("Failed to grab frame")
#         break


#     frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
#     results = hands.process(frame_rgb)

#     if results.multi_hand_landmarks:
#         for hand_landmarks in results.multi_hand_landmarks:
#             mp_drawing.draw_landmarks(
#                 frame,
#                 hand_landmarks,
#                 mp_hands.HAND_CONNECTIONS
#             )

#     cv2.imshow("Kinect Hand Tracking", frame)

#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# cap.release()
# cv2.destroyAllWindows()