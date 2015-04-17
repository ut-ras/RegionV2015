import cv2

lower = None
upper = None


capture = cv2.VideoCapture(0)

def captureFrameAndGetAvgHSV():
    ret,img = capture.read()
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    current = hsv[60,40][0]
    current += hsv[60,80][0]
    current += hsv[100,40][0]
    current += hsv[100,80][0]
    current = current/4 
    return current

def main():
    hue = captureFrameAndGetAvgHSV()
    lower = hue
    upper = hue
    while True:
        hue = captureFrameAndGetAvgHSV()
        if (hue < lower):
            lower = hue
        if (hue > upper):
            upper = hue
        print "lower: "
        print lower
        print "upper: "
        print upper


main()
