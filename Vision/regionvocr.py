import cv2
import os
import numpy as np

IMAGE_FILE = "temp.jpg"
TXT_FILE = "data"
TXT_EXTENSION = ".txt"

possibleChars = ['A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z','1','2','3','4','5','6','7','8','9','0','!','@','#','$','%','^','&','*','(',')']

lower_blue = np.array([70,50,50], dtype=np.uint8)
upper_blue = np.array([110,255,255], dtype=np.uint8)
lower_green = np.array([30,50,50], dtype=np.uint8)
upper_green = np.array([70,255,255], dtype=np.uint8)
lower_pink = np.array([0,50,50], dtype=np.uint8)
upper_pink = np.array([20,255,255], dtype=np.uint8)
lower_pinkk = np.array([140,50,50], dtype=np.uint8)
upper_pinkk = np.array([179,255,255], dtype=np.uint8)

def initCamera():
	#add these two lines for cam on robot (otherwise opencv times out when captureing image)
	os.system("rmmod uvcvideo")
	os.system("modprobe uvcvideo nodrop=1 timeout=5000")

	#capture = cv2.VideoCapture(0)

	#take out these two lines for cam on robot (needed for my webcam at home)
	#capture.set(3, 160)
	#capture.set(4, 120)
	
	#display images (comment out when running on beaglebone)
	#cv2.namedWindow('capture', 0)

def captureFrame():
	capture = cv2.VideoCapture(0)
	ret,img = capture.read()
	if(ret == False):
		print "capture failed"
	else:
		print "image capture"

	#comment out imshow when running on beaglebone 
	#cv2.imshow('capture', img)
	print "writing img"
	cv2.imwrite(IMAGE_FILE, img) 
	capture.release()
	return img

def getColor(img):
	img = cv2.blur(img,(3,3))
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	threshBlue = cv2.inRange(hsv, lower_blue, upper_blue)
	threshGreen = cv2.inRange(hsv, lower_green, upper_green)
	threshPink = cv2.inRange(hsv, lower_pink, upper_pink)
	threshPinkk = cv2.inRange(hsv, lower_pinkk, upper_pinkk)

	totalBlue = cv2.countNonZero(threshBlue)
	totalGreen = cv2.countNonZero(threshGreen)
	totalPink = cv2.countNonZero(threshPink) + cv2.countNonZero(threshPinkk)
	
	if (totalBlue > totalPink) and (totalBlue > totalGreen):
		print "panel is blue"
		return "blue"
	elif (totalPink > totalGreen):
		print "panel is pink"
		return "pink"
	else:
		print "panel is green"
		return "green"

def getCharacterFromImageFile(color):
	#characters file must be place in tesseract config folder ( /tesseract/share/tessdata/config/)
	if color == "blue":
		os.system("tesseract " + IMAGE_FILE + " " + TXT_FILE + " -psm 10" + " bluecharacters")
	elif color == "pink":
		os.system("tesseract " + IMAGE_FILE + " " + TXT_FILE + " -psm 10" + " pinkcharacters")
	else:
		return None

	data = open(TXT_FILE + TXT_EXTENSION)
	character = data.read()
	data.close()
	#check if string is empty
	if not character:
		print "string from data.txt is empty"
		return None
	#check if its one of our possibleChars
	if character[0] in possibleChars:
		return character[0]
	else:
		print "character not on possible character list - " + character[0]
		return None

def main():
	initCamera() 
	while True:
		img = captureFrame()
		color = getColor(img)
		char = getCharacterFromImageFile(color)
		if char is not None:
			print "found: "
			print char
		else:
			print "nothing found"

		k = raw_input('hit enter to continue')
		if k == 'q':
			break

main()
