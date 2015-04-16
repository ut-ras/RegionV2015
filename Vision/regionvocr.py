import cv2
import os
import commands
from time import sleep

IMAGE_FILE = "temp.png"
TXT_FILE = "data"
TXT_EXTENSION = ".txt"

possibleChars = ['A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z','1','2','3','4','5','6','7','8','9','0','!','@','#','$','%','^','&','*','(',')']


def initCamera():
    #add these two lines for cam on robot (otherwise opencv times out when captureing image)
    os.system("rmmod uvcvideo")
    os.system("modprobe uvcvideo nodrop=1 timeout=5000")

    capture = cv2.VideoCapture(0)

    #take out these two lines for cam on robot (needed for my webcam at home)
    capture.set(3, 160)
    capture.set(4, 120)

    #display images (comment out when running on beaglebone)
    cv2.namedWindow('capture', 0)

    return capture

def captureFrame(capture):
    ret,img = capture.read()
    if(ret == False):
        print "capture failed"
        return ret

    #comment out imshow when running on beaglebone 
    cv2.imshow('capture', img)
    cv2.imwrite(IMAGE_FILE, img) 

    return ret

def getCharacterFromImageFile():
    #characters file must be place in tesseract config folder ( /tesseract/share/tessdata/config/)
    shret = commands.getoutput("tesseract " + IMAGE_FILE + " " + TXT_FILE + " -psm 10" + " characters")

    data = open(TXT_FILE + TXT_EXTENSION)
    character = data.read()
    #check if string is empty
    if not character:
        return None
    #check if its one of our possibleChars
    if character[0] in possibleChars:
        return character[0]
    else:
        return None     #use if is None: to check if return is null

def main():
    cap = initCamera()
    while True:
        ret = captureFrame(cap)
        sleep(2)

main()
