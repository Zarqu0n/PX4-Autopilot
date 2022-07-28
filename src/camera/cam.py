import cv2
wCam , hCam = 640,480

cap = cv2.VideoCapture(0)


while True:
    succes,img = cap.read()

    cv2.imshow("Cam",img)
    cv2.waitKey(1)