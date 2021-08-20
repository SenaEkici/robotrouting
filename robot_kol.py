import cv2
import urllib.request
import imutils
import numpy as np
import serial
import time
ser = serial.Serial()
ser.baudrate = 115200
ser.port = 'COM7'
ser.timeout = 1


kernel = np.ones((10, 10), np.uint8)
kernel2 = np.ones((13,13), np.uint8)
acix=1
aciy=0

url='http://192.168.1.106/cam-hi.jpg'
#fourcc = cv2.VideoWriter_fourcc(*'XVID')
#out = cv2.VideoWriter('output3.avi', fourcc, 20.0, (800,600))
ser.open()

ser.write(b"xx1s90b\n\r")
ser.close()


while True:
    imgResp=urllib.request.urlopen(url)
    imgNp=np.array(bytearray(imgResp.read()),dtype=np.uint8)
    img=cv2.imdecode(imgNp,-1)
    (h, w) = img.shape[:2]

    center = (int(w / 2),int(h / 2))
    cv2.circle(img,center,5,(255,255,255),cv2.FILLED)
    center_m = [int(w / 2),int(h / 2)]
    blur = cv2.medianBlur(img, 13)
    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    canny = cv2.Canny(blur, 10, 150)
    dilation = cv2.dilate(canny, kernel2, iterations=2)
    cnts = cv2.findContours(dilation, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    for c in cnts:
        M = cv2.moments(c)
        area = cv2.contourArea(c)
        (x, y, w, h) = cv2.boundingRect(c)
        # print(area)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 250), 2)
            cv2.circle(img, (cX, cY), 3, (255, 255, 255), -1)
            cv2.putText(img, str(cX) + "," + str(cY), (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255, 255, 255),2)
            cv2.line(img, center, (cX, cY), (255, 255, 255), 2)
            print(str(cX - center[0]))
            if -10 < cX - center_m[0] < 50:
                ser.open()
                ser.write(b"xxy\n\r")
                print("hey")
                time.sleep(5)
                ser.write(b"xx2s120b\n\r")
                time.sleep(5)
                ser.write(b"xx3s10b\n\r")
                time.sleep(5)
                ser.close()
                if -50 < cY - center_m[1] < 50:
                    ser.open()
                    ser.write(b"xx5s130b\n\r")
                    ser.close()
                else:
                    ser.open()
                    ser.write(b"xx2s130b\n\r")
                    ser.close()







    cv2.imshow('kamera', img)

    if ord('q')==cv2.waitKey(10):
        exit(0)


