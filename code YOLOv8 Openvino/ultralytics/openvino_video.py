import cv2
from ultralytics import YOLO
import time
import ipywidgets as widgets
import openvino.runtime as ov
model =  YOLO('yolov8n.pt') #can comment
model.export(format='openvino',dynamic=True,imgsz=(640,640),half=False) #can comment
core = ov.Core()
device = widgets.Dropdown(
    options=core.available_devices + ["GPU"],
    value='GPU',
    description='Device:',
    disabled=False,
)
device
ov_model = YOLO('yolov8n_openvino_model')
video_path = 'your_video_file.mp4'
cap = cv2.VideoCapture(video_path)
while cap.isOpened():
    success, frame = cap.read()
    start_time = time.time()
    results = ov_model(frame)
    end_time = time.time()
    print(f"Inference time: {end_time - start_time:.3f} seconds")
    annotated_frame = results[0].plot()
    cv2.imshow("YOLOv8 Inference", annotated_frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
cap.release()
cv2.destroyAllWindows()
