import cv2
import numpy as np
min_H,max_H=160,206
min_S,max_S=149,185
min_V,max_V=53,227
def onChange(x):
    global minBounds, maxBounds
    minBounds[0]= cv2.getTrackbarPos('minH', 'Settings')
    minBounds[1] = cv2.getTrackbarPos('minS', 'Settings')
    minBounds[2]= cv2.getTrackbarPos('minV', 'Settings')
    maxBounds[0] = cv2.getTrackbarPos('maxH', 'Settings')
    maxBounds[1] = cv2.getTrackbarPos('maxS', 'Settings')
    maxBounds[2] = cv2.getTrackbarPos('maxV', 'Settings')
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cv2.namedWindow('Settings')
cv2.createTrackbar('maxH', 'Settings', max_H, 255, onChange)
cv2.createTrackbar('minH', 'Settings', min_H, 255, onChange)
cv2.createTrackbar('maxS', 'Settings', max_S, 255, onChange)
cv2.createTrackbar('minS', 'Settings', min_S, 255,onChange)
cv2.createTrackbar('maxV', 'Settings', max_V, 255, onChange)
cv2.createTrackbar('minV', 'Settings', min_V, 255, onChange)
minBounds = np.array([min_H, min_S, min_V])
maxBounds = np.array([max_H, max_S, max_V])
while True:
    _, frame = cap.read()
    blur = cv2.medianBlur(frame,7)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, minBounds, maxBounds)
    res = cv2.bitwise_and(frame, frame, mask=mask)
    cv2.imshow('frame', frame)
    cv2.imshow('mask', mask)
    cv2.imshow('res', res)
    key = cv2.waitKey(1)
    if key == 27:
        break
cap.release()
cv2.destroyAllWindows()