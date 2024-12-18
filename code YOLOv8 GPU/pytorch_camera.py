import cv2  # OpenCV library
from ultralytics import YOLO  # YOLO library

# Load the YOLOv8 model
model = YOLO("yolov8n.pt")  # Replace with your YOLO model path if different

# Initialize video capture (0 for webcam, or replace with video file path)
cap = cv2.VideoCapture(0)  # Use 'video.mp4' for file input

while cap.isOpened():
    success, frame = cap.read()  # Capture frame
    if not success:
        print("Warning: Unable to read frame.")
        break

    # Perform YOLO inference
    results = model(frame)

    # Annotate the frame with detection results
    annotated_frame = results[0].plot()

    # Display the annotated frame
    cv2.imshow("YOLOv8 Inference", annotated_frame)

    # Exit the loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord("q"):
        print("Exiting...")
        break
finally:
    # Release resources
    cap.release()
    cv2.destroyAllWindows()
    print("Resources released. Program terminated.")
