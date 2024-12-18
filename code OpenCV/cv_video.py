import cv2
video_path = 'video.mp4'
cap = cv2.VideoCapture(video_path)
while True:
    ret, frame = cap.read()
    cv2.imshow('Video Playback', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
