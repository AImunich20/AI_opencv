import cv2
import mediapipe as mp
import numpy as np
import time  # To calculate FPS

# Mediapipe Face Mesh
mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(min_detection_confidence=0.6, min_tracking_confidence=0.6)

# Webcam
cap = cv2.VideoCapture(0)

# Eye and Mouth Landmark Indices
LEFT_EYE = [33, 160, 158, 133, 153, 144, 145, 23, 24]
RIGHT_EYE = [362, 385, 387, 263, 373, 380, 374, 253, 254]
UPPER_LIP = [61, 62, 63, 64, 65, 66]
LOWER_LIP = [146, 91, 181, 84, 17, 14]

# Constants
EAR_BASE_THRESHOLD = 0.25
yawn_frames = 0
mar_values = []
window_size = 10
MAR_NORMAL = []
MAR_THRESHOLD = 0.8

def calculate_aspect_ratio(points, indices):
    selected_points = np.array([[points[i].x, points[i].y] for i in indices])
    vertical_1 = np.linalg.norm(selected_points[1] - selected_points[5])
    vertical_2 = np.linalg.norm(selected_points[2] - selected_points[4])
    horizontal = np.linalg.norm(selected_points[0] - selected_points[3])

    if horizontal < 0.01:
        return 0.0
    return (vertical_1 + vertical_2) / (2.0 * horizontal)

def calculate_mouth_aspect_ratio(points):
    upper = np.mean([[points[i].x, points[i].y] for i in UPPER_LIP], axis=0)
    lower = np.mean([[points[i].x, points[i].y] for i in LOWER_LIP], axis=0)

    horizontal = np.linalg.norm(
        np.array([points[78].x, points[78].y]) - np.array([points[308].x, points[308].y])
    )
    vertical = np.linalg.norm(upper - lower)

    if horizontal < 0.01:
        return 0.0
    return vertical / horizontal

def adjust_ear_threshold(face_width, face_height, base_threshold):
    face_ratio = face_height / face_width if face_width > 0 else 1.0
    adjusted_threshold = base_threshold * (1 + (1 - face_ratio))
    return max(0.15, min(adjusted_threshold, 0.3))

while cap.isOpened():
    start_time = time.time()  # Start timer for FPS

    success, frame = cap.read()

    if success:
        frame = cv2.flip(frame, 1)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        face_results = face_mesh.process(rgb_frame)

        annotated_frame = frame.copy()

        if face_results.multi_face_landmarks:
            for face_landmarks in face_results.multi_face_landmarks:
                ih, iw, _ = frame.shape
                face_width = iw
                face_height = ih
                ear_threshold = adjust_ear_threshold(face_width, face_height, EAR_BASE_THRESHOLD)

                for landmark in face_landmarks.landmark:
                    x = int(landmark.x * iw)
                    y = int(landmark.y * ih)
                    cv2.circle(annotated_frame, (x, y), 1, (255, 255, 0), -1)

                try:
                    left_ear = calculate_aspect_ratio(face_landmarks.landmark, LEFT_EYE)
                    right_ear = calculate_aspect_ratio(face_landmarks.landmark, RIGHT_EYE)
                    avg_ear = (left_ear + right_ear) / 2.0

                    mar = calculate_mouth_aspect_ratio(face_landmarks.landmark)

                    if len(mar_values) >= window_size:
                        mar_values.pop(0)
                    mar_values.append(mar)
                    smoothed_mar = np.mean(mar_values)

                    if len(MAR_NORMAL) < 50:
                        MAR_NORMAL.append(mar)
                    else:
                        mean_mar = np.mean(MAR_NORMAL)
                        std_mar = np.std(MAR_NORMAL)
                        MAR_THRESHOLD = mean_mar + 2 * std_mar

                    # Display EAR, MAR, and FPS on the frame
                    cv2.putText(
                        annotated_frame,
                        f"EAR: {avg_ear:.2f}",
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2
                    )

                    cv2.putText(
                        annotated_frame,
                        f"MAR: {smoothed_mar:.2f}",
                        (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2
                    )

                    # Check for drowsiness and yawning
                    if avg_ear < ear_threshold:
                        cv2.putText(
                            annotated_frame,
                            "DROWSINESS DETECTED!",
                            (10, 70),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 0, 255),
                            2
                        )

                    if smoothed_mar > MAR_THRESHOLD:
                        yawn_frames += 1
                        if yawn_frames > 10:
                            cv2.putText(
                                annotated_frame,
                                "YAWNING DETECTED!",
                                (10, 90),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.5,
                                (255, 0, 255),
                                2
                            )
                    else:
                        yawn_frames = 0

                except IndexError:
                    print("Index out of range, skipping this face.")

        # Calculate FPS (frames per second)
        elapsed_time = time.time() - start_time
        fps = 1 / elapsed_time

        # Display FPS on the frame
        cv2.putText(
            annotated_frame,
            f"FPS: {fps:.2f}",
            (10, 110),  # Adjust the position to avoid overlap with other text
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 0),
            2
        )

        cv2.imshow("Face, Eye & Yawn Detection", annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        break

cap.release()
cv2.destroyAllWindows()
