import cv2
import numpy as np

def main():
    cv2.namedWindow('threshB', 0)
    cv2.namedWindow('threshG', 0)
    cv2.namedWindow('threshP', 0)
    cv2.namedWindow('threshPk', 0)
    cap = cv2.VideoCapture(0)
    cap.set(3, 160)
    cap.set(4, 120)
    ret,img = cap.read()
    img = cv2.blur(img,(3,3))

    lower_blue = np.array([85,50,50], dtype=np.uint8)
    upper_blue = np.array([125,255,255], dtype=np.uint8)
    lower_green = np.array([40,50,50], dtype=np.uint8)
    upper_green = np.array([80,255,255], dtype=np.uint8)
    lower_pink = np.array([0,50,50], dtype=np.uint8)
    upper_pink = np.array([20,255,255], dtype=np.uint8)
    lower_pinkk = np.array([140,50,50], dtype=np.uint8)
    upper_pinkk = np.array([179,255,255], dtype=np.uint8)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    threshBlue = cv2.inRange(hsv, lower_blue, upper_blue)
    threshGreen = cv2.inRange(hsv, lower_green, upper_green)
    threshPink = cv2.inRange(hsv, lower_pink, upper_pink)
    threshPinkk = cv2.inRange(hsv, lower_pinkk, upper_pinkk)
    print "countNonZeroBlue: "
    print cv2.countNonZero(threshBlue)
    print "countNonZeroGreen: "
    print cv2.countNonZero(threshGreen)
    print "countNonZeroPink: "
    print cv2.countNonZero(threshPink) + cv2.countNonZero(threshPinkk)

    cv2.imshow('threshB', threshBlue)
    cv2.imshow('threshG', threshGreen)
    cv2.imshow('threshP', threshPink)
    cv2.imshow('threshPk', threshPinkk)
    c = cv2.waitKey(0)
    if c == 27:
        cv2.destroyAllWindows()

main()
