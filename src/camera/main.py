import cv2
import time
import os
import HandTrackingModule as htm
wCam , hCam = 640,480

cap = cv2.VideoCapture(0)
cap.set(3,wCam)
cap.set(4,hCam)

tipIds = [4,8,12,16,20]
detector = htm.handDetector(detectionCon=0.75)
while True:
    succes,img = cap.read()
    img = detector.findHands(img)
    lmlist = detector.findPosition(img,draw=False)

    if len(lmlist) != 0:
        x = 0
        y = 0
        for i in range(0,20):
            x = x + lmlist[i][1]
            y = y + lmlist[i][2]
        x = x//20
        y = y//20
        img= cv2.circle(img, (x,y), 10, (0,0,0), 5)
        fingers = []
        #print(x,y)
        if lmlist[tipIds[0]][1] < lmlist[tipIds[0] - 1][1]:
            fingers.append(1)
        else:
            fingers.append(0)
        for id in range(1,5):
            if lmlist[tipIds[id]][2] < lmlist[tipIds[id]-2][2]:
                fingers.append(1)
            else:
                fingers.append(0)

        totalfingers = fingers.count(1)
        #print(totalfingers)
    img = cv2.rectangle(img,(480,432),(160,48),(0,0,0),1)
    cv2.imshow("Cam",img)
    cv2.waitKey(1)