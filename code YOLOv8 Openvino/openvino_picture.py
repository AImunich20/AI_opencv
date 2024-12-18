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
image_path = 'your_image_file.jpg'
image = cv2.imread(image_path)
results = ov_model(image)
annotated_image = results[0].plot()
cv2.imshow("YOLOv8 Inference", annotated_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
