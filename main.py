import cv2 as cv

cap = cv.VideoCapture('assets\dashcam637cut\part_000.mkv')

while cap.isOpened():
    ret, frame = cap.read()

    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    frame_resized = cv.resize(gray_frame, (640, 640))
    cv.imshow('frame', frame_resized)
    if cv.waitKey(50) == ord('q'):
        break

cap.release()
cv.destroyAllWindows()