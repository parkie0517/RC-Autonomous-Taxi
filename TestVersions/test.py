import numpy as np 
import cv2


cap = cv2.VideoCapture(1)

ret, frame = cap.read()

while True:
    ret, frame = cap.read() # frame shape is (480, 640, 3)
    frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)


    if cv2.waitKey(1) == ord('q'):
        break
    
    cv2.imshow('After', frame)

cv2.imwrite('frame.jpg', frame)

cap.release()

cv2.destroyAllWindows()