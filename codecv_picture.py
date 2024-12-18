import cv2
image_path = 'image.jpg'
image = cv2.imread(image_path)
cv2.imshow('Image Viewer', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
