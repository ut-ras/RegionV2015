import cv2

cap = cv2.VideoCapture(0)

cap.set(3, 160)
cap.set(4,120)

ret, img = cap.read()

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

print hsv[40,40]
print hsv[100,100]
