import cv2
import mediapipe as mp
import numpy as np
import math


class GestureDetector:
    def __init__(self):
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5)

        # Get hand connections for drawing wireframe
        self.mp_drawing = mp.solutions.drawing_utils
        self.hand_connections = self.mp_hands.HAND_CONNECTIONS

    def detect(self, frame):
        # Mirror the frame
        frame = cv2.flip(frame, 1)

        # Convert BGR to RGB
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_frame)

        debug_frame = frame.copy()


        if results.multi_hand_landmarks:
            hand_landmarks = results.multi_hand_landmarks[0]
            landmarks = hand_landmarks.landmark
            h, w = frame.shape[:2]

            wrist = landmarks[0]



            # Finger tips (mediapipe indices)
            tips = {
                'index': landmarks[8],
                'middle': landmarks[12],
                'ring': landmarks[16],
                'pinky': landmarks[20]
            }

            # - value to more raised
            # + value to more folded (increase the value)
            # Will implement dynamic positions later.

            # Finger states (True = extended, False = folded)
            finger_states = {
                'index': tips['index'].y < wrist.y - 0.3,
                'middle': tips['middle'].y < wrist.y - 0.3,
                'ring': tips['ring'].y > wrist.y + 0.05,  # Folded when ring is below wrist
                'pinky': tips['pinky'].y > wrist.y -0.1
            }

            # Get coordinates for drawing
            middle_x, middle_y = int(tips['middle'].x * w), int(tips['middle'].y * h)

            # Draw wireframe skeleton (white lines)
            self.mp_drawing.draw_landmarks(
                debug_frame,
                hand_landmarks,
                self.hand_connections,
                connection_drawing_spec=self.mp_drawing.DrawingSpec(
                    color=(255, 255, 255),  # White color
                    thickness=2,  # Line thickness
                    circle_radius=0  # Hide the circles
                ),
                landmark_drawing_spec=None  # Hide the landmarks
            )

            # Draw landmarks
            for name, tip in tips.items():
                color = (0, 255, 0) if finger_states[name] else (0, 0, 255)
                cv2.circle(debug_frame,
                           (int(tip.x * w), int(tip.y * h)),
                           10, color, -1)

            # Gesture Logic Below -------------------------------------------------

            # Case 1. Only middle finger up (strict full erase)
            if (finger_states['middle'] and
                    not finger_states['index'] and
                    not finger_states['ring'] and
                    not finger_states['pinky']):

                cv2.putText(debug_frame, "FULL ERASE", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                return "full_erase", middle_x, middle_y, debug_frame

            # Case 2. Fist (all fingers folded)
            elif (not finger_states['index'] and
                  not finger_states['middle'] and
                  not finger_states['ring'] and
                  not finger_states['pinky']):

                cv2.putText(debug_frame, "ERASE", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                return "erase", middle_x, middle_y, debug_frame

            # 3. Index + Pinky only (color change)
            elif (finger_states['index'] and
                  not finger_states['middle'] and
                  not finger_states['ring'] and
                  finger_states['pinky']):

                cv2.putText(debug_frame, "CHANGE COLOR", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
                return "change_color", middle_x, middle_y, debug_frame

            # 4. Default painting (just index finger)
            elif (finger_states['index'] and
                  not finger_states['middle'] and
                  not finger_states['ring'] and
                  not finger_states['pinky']):

                cv2.putText(debug_frame, "PAINT", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                return "paint", int(tips['index'].x * w), int(tips['index'].y * h), debug_frame

        return "none", 0, 0, debug_frame