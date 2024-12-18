import cv2  # OpenCV library
from ultralytics import YOLO  # YOLO library

# Load the YOLOv8 model
model = YOLO("yolov8n.pt")  # Replace with your YOLO model path if different

# Replace 'path_to_your_video.mp4' with the actual path to your video file
video_path = "path_to_your_video.mp4"
cap = cv2.VideoCapture(video_path)

while cap.isOpened():
    success, frame = cap.read()  # Read a frame from the video
    if not success:
        print("End of video or unable to read frame.")
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

# Release resources
cap.release()
cv2.destroyAllWindows()
print("Resources released. Program terminated.")
