import cv2  # OpenCV library
from ultralytics import YOLO  # YOLO library

# Load the YOLOv8 model
model = YOLO("yolov8n.pt")  # Replace with your YOLO model path if different

# Load the image
image_path = "path_to_your_image.jpg"  # Replace with your image path
frame = cv2.imread(image_path)

if frame is not None:  # Check if the image is loaded successfully
    # Perform YOLO inference
    results = model(frame)

    # Annotate the image with results
    annotated_frame = results[0].plot()

    # Display the annotated image
    cv2.imshow("YOLOv8 Inference", annotated_frame)

    # Wait until a key is pressed or for 5 seconds (5000 milliseconds)
    cv2.waitKey(5000)

    # Optionally save the annotated image to a file
    output_path = "annotated_image.jpg"  # Specify the path to save the image
    cv2.imwrite(output_path, annotated_frame)

    print(f"Annotated image saved at: {output_path}")
else:
    print("Error: Could not load the image. Check the image path.")

cv2.destroyAllWindows()