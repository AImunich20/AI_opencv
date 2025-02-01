import cv2
from ultralytics import YOLO
import mediapipe as mp
import numpy as np
import openvino.runtime as ov
import requests
import pyfirmata
import time

arduino_ip = "192.168.0.51"  # เปลี่ยนเป็น IP ของ Arduino
url = f"http://{arduino_ip}"
last_data = None  # ตัวแปรเก็บข้อมูลล่าสุดที่ได้รับ

# # โหลดโมเดล YOLO
model = YOLO("yolov8n.pt")  # ใช้ YOLOv8n สำหรับตรวจจับคน

def lineNotify(message, image):
    url = 'https://notify-api.line.me/api/notify'
    token = 'pO4cJOwZCR6iSnveTz9MzR1z61jdvTAqW2howZoVX4R'
    headers = {'Authorization': 'Bearer ' + token}
    payload = {'message': message}
    files = {'imageFile': image}
    requests.post(url, headers=headers, data=payload, files=files)

core = ov.Core()

# Arduino setup
port = 'COM13'
board = pyfirmata.Arduino(port)
LED_pin1 = board.get_pin('d:2:o')
LED_pin2 = board.get_pin('d:3:o')

HIGH = True  # Turn on LED
LOW = False  # Turn off LED

# Mediapipe setup
mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(min_detection_confidence=0.6, min_tracking_confidence=0.6)

# Webcam setup
cap = cv2.VideoCapture(0)
cap2 = cv2.VideoCapture(1)

# Eye and lip indices for face mesh
LEFT_EYE = [33, 160, 158, 133, 153, 144, 145, 23, 24]
RIGHT_EYE = [362, 385, 387, 263, 373, 380, 374, 253, 254]
UPPER_LIP = [61, 62, 63, 64, 65, 66]
LOWER_LIP = [146, 91, 181, 84, 17, 14]

# Initial values
EAR_BASE_THRESHOLD = 0.25
yawn_frames = 0
mar_values = []
window_size = 10  # For Moving Average
MAR_NORMAL = []
MAR_THRESHOLD = 0.8  # Default threshold

drowsiness_start_time = None  # Start time for detecting drowsiness
drowsiness_threshold_time = 2  # 2 seconds threshold
time_line = 3  # Line time threshold
pupil_stability_start_time = None
pupil_stability_threshold_time = 10  # 10 seconds for pupil stability
last_pupil_center = None  # Initialize the last pupil position

# Function to calculate Eye Aspect Ratio (EAR)
def calculate_aspect_ratio(points, indices):
    selected_points = np.array([[points[i].x, points[i].y] for i in indices])
    vertical_1 = np.linalg.norm(selected_points[1] - selected_points[5])
    vertical_2 = np.linalg.norm(selected_points[2] - selected_points[4])
    horizontal = np.linalg.norm(selected_points[0] - selected_points[3])

    if horizontal < 0.01:
        return 0.0
    return (vertical_1 + vertical_2) / (2.0 * horizontal)

# Function to calculate Mouth Aspect Ratio (MAR)
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

# Function to adjust EAR threshold based on face size
def adjust_ear_threshold(face_width, face_height, base_threshold):
    face_ratio = face_height / face_width if face_width > 0 else 1.0
    adjusted_threshold = base_threshold * (1 + (1 - face_ratio))
    return max(0.15, min(adjusted_threshold, 0.3))

# Function to fetch data from the server
def fetch_json_data(url, last_data=None, timeout=1):
    try:
        response = requests.get(url, timeout=timeout)
        response.raise_for_status()
        
        if "application/json" in response.headers.get("Content-Type", ""):
            data = response.json()
            if data != last_data:
                print("Received Data:", data)
                lineNotify(str(data), None)  # Send data to LINE
                return data
        else:
            print("Error: Response is not JSON")
            print(response.text)
    except requests.RequestException as e:
        print("Request Error:", e)
    
    return last_data  # Return the last data if error occurs

while cap.isOpened():
    success, frame = cap.read()
    if success:
        frame = cv2.flip(frame, 1)  # กลับด้านภาพ
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = model(frame)  # ตรวจจับคนด้วย YOLO
        annotated_frame = frame.copy()

        max_area = 0
        largest_person_box = None

        # ตรวจจับคนในภาพ
        for r in results:
            boxes = r.boxes
            for box in boxes:
                cls = int(box.cls)
                label = model.names[cls]

                if label == 'person':
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    area = (x2 - x1) * (y2 - y1)

                    if area > max_area:
                        max_area = area
                        largest_person_box = (x1, y1, x2, y2)

        if largest_person_box:
            x1, y1, x2, y2 = largest_person_box
            person_roi = frame[y1:y2, x1:x2]
            

            if person_roi.size > 0:
                rgb_roi = cv2.cvtColor(person_roi, cv2.COLOR_BGR2RGB)
                face_results = face_mesh.process(rgb_roi)

                if face_results.multi_face_landmarks:
                    for face_landmarks in face_results.multi_face_landmarks:
                        ih, iw, _ = person_roi.shape

                        # คำนวณขนาดใบหน้า
                        face_width = x2 - x1
                        face_height = y2 - y1

                        # ปรับค่า EAR Threshold
                        ear_threshold = adjust_ear_threshold(face_width, face_height, EAR_BASE_THRESHOLD)

                        # วาดจุด Face Mesh
                        for landmark in face_landmarks.landmark:
                            x = int(landmark.x * iw) + x1
                            y = int(landmark.y * ih) + y1
                            cv2.circle(annotated_frame, (x, y), 1, (255, 255, 0), -1)

                        try:
                            # คำนวณ EAR
                            left_ear = calculate_aspect_ratio(face_landmarks.landmark, LEFT_EYE)
                            right_ear = calculate_aspect_ratio(face_landmarks.landmark, RIGHT_EYE)
                            avg_ear = (left_ear + right_ear) / 2.0

                            # คำนวณ MAR
                            mar = calculate_mouth_aspect_ratio(face_landmarks.landmark)

                            # ใช้ Moving Average ลด Noise
                            if len(mar_values) >= window_size:
                                mar_values.pop(0)
                            mar_values.append(mar)
                            smoothed_mar = np.mean(mar_values)
    
                            # เก็บข้อมูล MAR ปกติในช่วงแรก
                            if len(MAR_NORMAL) < 50:
                                MAR_NORMAL.append(mar)
                            else:
                                mean_mar = np.mean(MAR_NORMAL)
                                std_mar = np.std(MAR_NORMAL)
                                MAR_THRESHOLD = mean_mar + 2 * std_mar

                            # แสดงค่า EAR และ MAR ที่มุมซ้ายบนของจอ
                            cv2.putText(
                                annotated_frame,
                                f"EAR: {avg_ear:.2f} MAR: {smoothed_mar:.2f}",
                                (10, 30),  # ตำแหน่งมุมซ้ายบนของจอ
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.5,
                                (0, 255, 0),
                                2
                            )

                            # ตรวจจับการหลับใน
                            if avg_ear < ear_threshold:
                                # แสดงข้อความ DROWSINESS DETECTED!
                                cv2.putText(
                                    annotated_frame,
                                    "DROWSINESS DETECTED!",
                                    (10, 50),  # ตำแหน่งมุมซ้ายบนของจอ
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.5,
                                    (0, 0, 255),
                                    2
                                )
                
                                if drowsiness_start_time is None:
                                    drowsiness_start_time = time.time()
                                elapsed_time = time.time() - drowsiness_start_time
                                if elapsed_time >= drowsiness_threshold_time:
                                    LED_pin1.write(HIGH) 
                                if (elapsed_time >= 3):
                                    LED_pin2.write(HIGH) 
                                    _, img_encoded = cv2.imencode('.jpg', annotated_frame)
                                    lineNotify("DROWSINESS DETECTED!", img_encoded.tobytes())
                                    LED_pin1.write(LOW)
                                    LED_pin2.write(LOW)
                                if (elapsed_time >= time_line):
                                    LED_pin1.write(HIGH)
                                    LED_pin2.write(HIGH)
                            else:
                                # รีเซ็ตเวลาหาก avg_ear >= ear_threshold
                                drowsiness_start_time = None
                                LED_pin1.write(LOW)
                                LED_pin2.write(LOW)

                            # ตรวจจับการหาว
                            if smoothed_mar > MAR_THRESHOLD:
                                yawn_frames += 1
                                if yawn_frames > 10:
                                    cv2.putText(
                                        annotated_frame,
                                        "YAWNING DETECTED!",
                                        (10, 70),  # ตำแหน่งมุมซ้ายบนของจอ
                                        cv2.FONT_HERSHEY_SIMPLEX,
                                        0.5,
                                        (255, 0, 255),
                                        2
                                    )
                            else:
                                yawn_frames = 0

                            # ตรวจจับการนิ่งของลูกตาดำ
                            left_pupil_center = np.mean([[face_landmarks.landmark[i].x, face_landmarks.landmark[i].y] for i in LEFT_EYE], axis=0)
                            right_pupil_center = np.mean([[face_landmarks.landmark[i].x, face_landmarks.landmark[i].y] for i in RIGHT_EYE], axis=0)
                            pupil_center = (left_pupil_center + right_pupil_center) / 2

                            if pupil_stability_start_time is None:
                                pupil_stability_start_time = time.time()
                            elapsed_time = time.time() - pupil_stability_start_time

                            if elapsed_time >= pupil_stability_threshold_time:
                                # ตรวจสอบว่าลูกตาดำมีการเคลื่อนไหวหรือไม่
                                if np.linalg.norm(pupil_center - last_pupil_center) < 0.01:  # ค่า 0.01 คือค่าที่กำหนด
                                    cv2.putText(
                                        annotated_frame,
                                        "PUPIL STABILITY DETECTED!",
                                        (10, 90),  # ตำแหน่งมุมซ้ายบนของจอ
                                        cv2.FONT_HERSHEY_SIMPLEX,
                                        0.5,
                                        (0, 255, 255),
                                        2
                                    )
                                    # ... (โค้ดแจ้งเตือน)
                                    if elapsed_time >= (pupil_stability_threshold_time+3):
                                        LED_pin1.write(HIGH) 
                                    if elapsed_time >= (pupil_stability_threshold_time+6):
                                        LED_pin2.write(HIGH)
                                        _, img_encoded = cv2.imencode('.jpg', annotated_frame)
                                        lineNotify("PUPIL STABILITY DETECTED!", img_encoded.tobytes())
                                        LED_pin1.write(LOW)
                                        LED_pin2.write(LOW)
                                else:
                                    pupil_stability_start_time = time.time()  # รีเซ็ตเวลา
                                    LED_pin1.write(LOW)
                                    LED_pin2.write(LOW)

                            last_pupil_center = pupil_center
                        except IndexError:
                            print("Index out of range, skipping this face.")

        else:
            # ถ้าไม่พบคนในกล้องตัวแรก
            success2, frame2 = cap2.read()
            
            if success2:
                results2 = model(frame2)
                frame2 = results2[0].plot()
                cv2.imshow("Camera 2", frame2)  # แสดงผลจากกล้องตัวที่ 2
                last_data = fetch_json_data(url, last_data)
                for r in results:
                    boxes = r.boxes
                    for box in boxes:
                        cls = int(box.cls)
                        label1 = model.names[cls]

                        if label1 == 'person':
                            LED_pin2.write(HIGH) 
                            _, img_encoded = cv2.imencode('.jpg', annotated_frame)
                            lineNotify("Have people in this car!", img_encoded.tobytes())
                            LED_pin1.write(LOW)

        # แสดงผล
        cv2.imshow("YOLO + Face, Eye & Yawn Detection", annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):  # กด 'q' เพื่อออกจากโปรแกรม
            break
    else:
        break

cap.release()
cv2.destroyAllWindows()
board.exit()
