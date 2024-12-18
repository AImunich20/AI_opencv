import cv2
from ultralytics import YOLO
model = YOLO("yolov8n.pt")
image_path = "path_to_your_image.jpg"
frame = cv2.imread(image_path)
if frame is not None:
    results = model(frame)
    annotated_frame = results[0].plot()
    cv2.imshow("YOLOv8 Inference", annotated_frame)
    cv2.waitKey(5000)
    output_path = "annotated_image.jpg"
    cv2.imwrite(output_path, annotated_frame)
    print(f"Annotated image saved at: {output_path}")
else:
    print("Error: Could not load the image. Check the image path.")
cv2.destroyAllWindows()