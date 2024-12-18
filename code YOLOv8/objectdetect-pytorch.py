import cv2 #opencv pip install opencv-python 
from ultralytics import YOLO #pip install ultralytics YOLO,torch,torch vision 

model = YOLO("yolov8n.pt") #.pytorch .pt modelYOLO

cap = cv2.VideoCapture(0)#cv2.Video(1.mp4) cv2.VideoCapture(0)

while cap.isOpened():#loop cap open?

    success, frame = cap.read()#cap= 2frame

    if success:

        results = model(frame)
        annotated_frame = results[0].plot()
        cv2.imshow("YOLOv8 Inference", annotated_frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    else:
        break

cap.release()
cv2.destroyAllWindows()

