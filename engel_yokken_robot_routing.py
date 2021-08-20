import cv2
import numpy as np
import imutils
import math
import time
import serial
from itertools import permutations

ser = serial.Serial()
ser.baudrate = 115200
ser.port = 'COM7'
ser.timeout=1
kernel = np.ones((10, 10), np.uint8)
kernel2 = np.ones((13,13), np.uint8)
centers = []
cop = []
robot_center = []
#strt_point = [[145, 370]]
cap = cv2.VideoCapture(1)
def oklid_uzaklik( cX1, cY1, cX2, cY2):
    D = math.sqrt((cX2 - cX1) ** 2 + (cY2 - cY1) ** 2)
    D = int(D)
    return D
def led(frame,img1,robot_center):
    led_merkez=[]
    blue_lower = np.array([120, 100, 100], np.uint8)
    blue_upper = np.array([130, 255, 255], np.uint8)
    hsv=cv2.cvtColor(img1,cv2.COLOR_BGR2HSV)
    mavi=cv2.inRange(hsv,blue_lower,blue_upper)
    dilation=cv2.dilate(mavi,np.ones((5, 5), np.uint8),iterations=4)
    cnts = cv2.findContours(dilation, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    cv2.imshow("mavi",dilation)
    for c in cnts:
        M = cv2.moments(c)
        area = cv2.contourArea(c)
        (x, y, w, h) = cv2.boundingRect(c)
        # print(area)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            led_merkez=[cX,cY]
    if len(led_merkez) > 0:
        cv2.line(frame, (robot_center[0][0], robot_center[0][1]), (led_merkez[0], led_merkez[1]), (0, 150, 240), 2)
        cv2.line(frame, (robot_center[0][0], 0), (robot_center[0][0], 480), (255, 255, 255), 1)
        cv2.line(frame, (0, robot_center[0][1]), (640, robot_center[0][1]), (255, 255, 255), 1)
    return led_merkez
def merkez(cnts, frame):
    for c in cnts:
        M = cv2.moments(c)
        area = cv2.contourArea(c)
        (x, y, w, h) = cv2.boundingRect(c)
        # print(area)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            if area > 10000:
                robot_center.append([cX, cY])
                # print(robot_center)
                cv2.putText(frame, "robot", (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 250), 2)
            else:
                cop.append([cX,cY])
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        else:
            cX, cY = 0, 0
        # cv2.drawContours(frame, [c], -1, (0, 255, 0), 2)
        cv2.circle(frame, (cX, cY), 3, (255, 255, 255), -1)
        cv2.putText(frame, str(cX) + "," + str(cY), (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        centers.append([cX, cY])
def aci_hesap(cop,led_merkez,kucuk_index):

    egim1 = (-robot_center[0][1] + led_merkez[1]) / (robot_center[0][0] - led_merkez[0])
    teta1 = int(math.atan(egim1) * 180 / (math.pi))
    egim2 = (-robot_center[0][1] + cop[kucuk_index][1]) / (robot_center[0][0] - cop[kucuk_index][0])
    teta2 = int(math.atan(egim2) * 180 / (math.pi))
    if robot_center[0][1] < cop[kucuk_index][1]: #cop asagıda
        if teta2 >= 0:
            teta2 = teta2
        if teta2 < 0:
            teta2 = teta2 + 180

    if robot_center[0][1] > cop[kucuk_index][1]: #cop yukarda
        if teta2 >= 0:
            teta2 = teta2 + 180
        if teta2 < 0:
            teta2 = teta2 + 360

    if robot_center[0][1] < led_merkez[1]: #robot asagı bakıyor
        if teta1 >= 0:
            teta1 = teta1
        if teta1 < 0:
            teta1 = teta1 + 180

    if robot_center[0][1] > led_merkez[1]:  #robot yukarı bakıyor
        if teta1 >= 0:
            teta1 = teta1 + 180
        if teta1 < 0:
            teta1 = teta1 + 360
    aci = teta2 - teta1
    if aci < 0:
        aci = 360 + aci
    return aci
def hareket(kucuk_index,d,led_merkez):
    if len(cop) > 0:
        aci = aci_hesap(cop, led_merkez, kucuk_index)
        mesafe = int(d * 0.193) - 22
        print("aci=" + str(aci))
        print("mesafe=" + str(mesafe))
        if aci > 10:
            if mesafe > 6:
                ser.open()
                ser.write(b"xx" + str(mesafe).encode() + b"a" + str(aci).encode() + b"b\n\r")
                ser.close()
                time.sleep(15)
def cop_cop_uzaklik(frame,cop):
    for i in range(0, len(cop)):
        for j in range(0,len(cop)):
          cv2.line(frame, (cop[i][0], cop[i][1]), (cop[j][0], cop[j][1]), (0, 0, 0), 2)
def rota_belirleme(frame):
    cop_cop_uzaklik(frame, cop)
    k = len(cop) + 1
    n = math.factorial(len(cop))
    d = [0] * len(cop)
    rota = np.zeros((n, k))
    indeks_comb = []
    yol = [0] * n
    for i in range(0, len(cop)):
        cv2.putText(frame, "cop" + str(i), (cop[i][0] + 20, cop[i][1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255),
                    2)
        cv2.line(frame, (robot_center[0][0], robot_center[0][1]), (cop[i][0], cop[i][1]), (0, 0, 0), 1)

    c_comb = permutations(cop)
    c_comb_ind = permutations(np.arange(len(cop)))
    for i in list(c_comb):
        print(i)
    for i in list(c_comb_ind):
        indeks_comb.append(i)
    array = np.array(indeks_comb)
    #print(array)
    for i in range(0, n):
        rota[i][0] = oklid_uzaklik(robot_center[0][0], robot_center[0][1], cop[array[i][0]][0],
                                   cop[array[i][0]][1])
        for j in range(1, len(cop)):
            rota[i][j] = oklid_uzaklik(cop[array[i][j - 1]][0], cop[array[i][j - 1]][1], cop[array[i][j]][0],
                                       cop[array[i][j]][1])
        rota[i][len(cop)] = oklid_uzaklik(cop[array[i][len(cop) - 1]][0], cop[array[i][len(cop) - 1]][1],
                                          robot_center[0][0], robot_center[0][1])
    #print(rota)
    total = np.sum(rota, axis=1)
    print(total.reshape(n, 1))
    total = total.reshape(n, 1)
    min_in = list(total).index(min(total))

    #print(min(total))
    #print(min_in)
    #print(rota[min_in])
    en_kisa_yol = rota[min_in]
    print(array[min_in])
    print(en_kisa_yol)
    #rota cizdirme işlemleri
    cv2.line(frame,(robot_center[0][0],robot_center[0][1]),(cop[array[min_in][0]][0],cop[array[min_in][0]][1]),(0,255,255),3)
    for i in range(1,len(cop)):
        cv2.line(frame, (cop[array[min_in][i-1]][0],cop[array[min_in][i-1]][1]), (cop[array[min_in][i]][0], cop[array[min_in][i]][1]),
                 (0, 255, 255), 3)

    cv2.line(frame, (cop[array[min_in][len(cop)-1]][0], cop[array[min_in][len(cop)-1]][1]), (robot_center[0][0],robot_center[0][1]),
             (0, 255, 255), 3)

    return array[min_in]
def geri_donus(led_merkez):
    ser.open()
    print("cop yok1")
    aci = aci_hesap([[led_merkez[0],400]], led_merkez, 0)
    d = oklid_uzaklik(robot_center[0][0], led_merkez[1], led_merkez[0], 400)
    mesafe = int(d * math.cos(aci * math.pi / 180) * 47 / 313)
    if 90 < aci < 270:
        mesafe = (mesafe * -1) + 35
    if 270 < aci < 360:
        mesafe = (mesafe) + 15
    print(str(aci) + " " + str(mesafe))
    ser.write(b"xx" + str(mesafe).encode() + b"a" + str(aci).encode() + b"b\n\r")
    time.sleep(5)
    ser.write(b"xxp0b\n\r")
    time.sleep(10)
    if led_merkez[1] > robot_center[0][1]:
        aci=270
        d = oklid_uzaklik(robot_center[0][0], robot_center[0][1], 30, 400)
        mesafe = (int(d * math.cos(aci * math.pi / 180) * 47 / 313)*-1)+25
        print(str(aci) + " " + str(mesafe))
        ser.write(b"xx" + str(mesafe).encode() + b"a" + str(aci).encode() + b"b\n\r")
        time.sleep(5)
        ser.write(b"xx0a270b\n\r")
        time.sleep(5)
        ser.write(b"xx0a270b\n\r")
    else:
        aci=90
        d = oklid_uzaklik( led_merkez[0], led_merkez[1], 30, 400)
        mesafe = (int(d * math.cos(aci * math.pi / 180) * 47 / 313)*-1)+25
        print(str(aci) + " " + str(mesafe))
        ser.write(b"xx" + str(mesafe).encode() + b"a" + str(aci).encode() + b"b\n\r")
        time.sleep(5)
        ser.write(b"xx0a270b\n\r")
        time.sleep(5)
        ser.write(b"xx0a270b\n\r")
    ser.close()
    time.sleep(50)
def hareket(mesafe,aci):
    ser.open()
    ser.write(b"xx" + str(mesafe).encode() + b"a" + str(aci).encode() + b"b\n\r")
    ser.close()
    time.sleep(5)

def main():
    ind=0
    while True:
        ret, frame = cap.read()
        blur = cv2.medianBlur(frame, 13)
        gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
        canny = cv2.Canny(blur, 10, 150)
        dilation = cv2.dilate(canny, kernel2, iterations=2)
        gradient = cv2.morphologyEx(dilation, cv2.MORPH_GRADIENT, kernel)
        cnts = cv2.findContours(gradient, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        cv2.circle(frame,(145,370),5,(0,0,255),cv2.FILLED)
        cv2.putText(frame, "START", (140, 385), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255),2)
        merkez(cnts, frame)
        time.sleep(1)
        if len(centers) > 5:
            del centers[:]
        if len(robot_center) > 1:
            del robot_center[:1]
        if len(cop) > 3:
            del cop[:]
        led_merkez = led(frame, blur, robot_center)

        if len(robot_center) == 1:
            if len(cop) > 0:
                print("robot merkez=" + str(robot_center))
                print("cop merkez="+str(cop))

                cop_sirasi=rota_belirleme(frame)
                ind += 1
                print("ind=" + str(ind))
                if ind == 8:
                    ind = 0
                    ser.open()
                    ser.write(b"xxp0b\n\r")
                    ser.close()
                if len(led_merkez)>0:
                    aci = aci_hesap(cop, led_merkez, cop_sirasi[0])
                    d = oklid_uzaklik(robot_center[0][0], robot_center[0][1], cop[0][0], cop[0][1])
                    mesafe = int(d * 0.193) - 22
                    print("aci=" + str(aci))
                    print("mesafe=" + str(mesafe))

                    if mesafe > 6:
                        hareket(mesafe,aci)
                    else:
                        geri_donus(led_merkez)


        cv2.imshow("kamera", frame)
        key = cv2.waitKey(1)
        if key == 27:
            break
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()