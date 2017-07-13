#!/usr/bin/python
# -*- coding: UTF-8 -*-

from __future__ import print_function
from threading import Thread
from shape_detector import ShapeDetector
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil
from time import sleep, time
import numpy as np
import cv2
import imutils
import math
import serial

### Fonksiyon ve sınıf tanımları

class WebcamVideoStream:
    def __init__(self, src = -1):

        self.stream = cv2.VideoCapture(src)
        (self.grabbed, self.frame) = self.stream.read()

        self.stopped = False

    def start(self):
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        while True:
            if self.stopped:
                return
            (self.grabbed, self.frame) = self.stream.read()

    def read(self):
        return self.frame

    def stop(self):
        self.stopped = True


def hazirlik_ve_kalkis(drone, irtifa):
    """
    Aracı yerden belirtilen irtifaya (metre) havalandırır. İrtifa kalkış noktasına göredir.
    """
    print("Basit hazırlık kontrolleri")
    # Don't try to arm until autopilot is ready
    while not drone.is_armable:
        print("Aracın başlaması bekleniyor...")
        sleep(1)

    print("Motorlar hazırlanıyor")
    # Copter should arm in GUIDED mode
    drone.mode    = VehicleMode("GUIDED")
    drone.armed   = True

    # Confirm vehicle armed before attempting to take off
    while not drone.armed:
        print( "Hazırlanma için bekleniyor...")
        sleep(1)

    ev_konum = drone.location.global_relative_frame

    print("Kalkış!")
    drone.simple_takeoff(irtifa) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" İrtifa: ", drone.location.global_relative_frame.alt)
        #Break and return from function just below target altitude.
        if drone.location.global_relative_frame.alt>=irtifa*0.95:
            print("Belirlenen irtifaya erişildi")
            break
        sleep(1)

    return ev_konum

def goruntu_isleme(grt):
    """
    Kameradan sağlanan görüntüyü (grt) verilen sınırlara göre inceleyerek matris renklerini belirler. Renklerin baş harflerini dizi olarak gönderir
    """

    hsv = cv2.cvtColor(grt, cv2.COLOR_BGR2HSV)
    rect_count = 0
    merkezler = []

    # Renk sınırları: (renk kodu, (hsv alt sınır, hsv üst sınır))
    renk_sinir = [
    ('k', (np.array([0, 50, 50], dtype = np.uint8) , np.array([12, 255, 255], dtype = np.uint8))), # Kırmızı
    ('m', (np.array([97, 80, 50], dtype = np.uint8) , np.array([105, 255, 255], dtype = np.uint8))), # Mavi
    ('s', (np.array([15, 175, 50], dtype = np.uint8) , np.array([30, 255, 255], dtype = np.uint8))), # Sarı
    ]

    for (renk, (alt, ust)) in renk_sinir:

        # Verilen rengi algıla ve şekilleri birleştir

        mask = cv2.inRange(hsv, alt, ust)

        blurred = cv2.GaussianBlur(mask, (15, 15), 0)
        thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]

        erndi = cv2.erode(thresh, None, iterations=2)
        erndi = cv2.dilate(erndi, None, iterations=2)

        # Resimdeki şekilleri belirle
        cnts = cv2.findContours(erndi.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if imutils.is_cv2() else cnts[1]
        sd = ShapeDetector()

        # Şekilleri tek tek incele
        if len(cnts) > 0:
            for c in cnts:
                shape = sd.detect(c)
                if (cv2.contourArea(c) > 1000 and (shape == 'sq' or shape == 'rect')): # Belirli bir boyuttan büyük karelere ve dikdörtgenlere odaklan

                    # Şeklin merkezini belirle
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                    else:
                        cX, cY = 0, 0

                    if renk == 'k':
                        color = (0, 0, 255)
                    elif renk == 'm':
                        color = (255, 0, 0)
                    elif renk == 's':
                        color = (0, 255, 255)

                    cv2.drawContours(grt, [c], -1, color, 2)

                    # Merkezler: şekillerin koordinatlarının ve renklerinin saklandığı liste
                    merkezler.append((cX, cY, renk))
                    rect_count += 1

        else:
            print('Hiçbir şekil algılanamadı')
            return 'Hata'

    #cv2.imshow("Image", grt)
    #cv2.waitKey(0)

    if rect_count == 16:
        merkezler = sirala(merkezler)
        renk_str = ''
        for (x, y, renk) in merkezler:
            renk_str = renk_str + renk
        print(merkezler)
        return renk_str

    else:
        print('16 kare bulunamadı')
        return 'Hata'

def sirala(matris):
    '''
    Verilen 16 kareyi konumlarına göre sıralandırır
    '''
    x_top = 0
    y_top = 0

    x_maks = 0
    x_min = 1024

    y_maks = 0
    y_min = 1024

    for (x, y, renk) in matris:
        x_top += x
        y_top += y

        if x < x_min:
            x = x_min
        if x > x_maks:
            x = x_maks
        if y < y_min:
            y = y_min
        if y > y_maks:
            y = y_maks

    x_mer = x_top / len(matris)
    y_mer = y_top / len(matris)

    orta_matris = []

    for (x, y, renk) in matris:
        orta_matris.append((int((x_mer-x)//((x_maks-x_min)/9)), int((y_mer-y)//((y_maks-y_min)/9)), renk))

#    orta_matris =

    return sorted(orta_matris, key=lambda koord: (koord[1], koord[0]))

def sd_kart(renk_str):
    '''
    Verilen metin dizisindeki (string) renklere göre metni SD karta yazar
    '''
    # ser = serial.Serial('/dev/ttyUSB0', 9600)
    # out_str = renk_str.replace('s', '0').replace('m', '1').replace('k', '2').encode()
    # ser.write(out_str)
    # sleep(0.2)
    # ser.close()
    print(renk_str)

def hedef_varis(drone, hedef):
    '''
    Drone'un belirlenen hedefe ulaşıp ulaşmadığını kontrol eder
    '''
    currentLocation = drone.location.global_relative_frame
    currentLocationLat = drone.location.global_relative_frame.lat
    currentLocationLon = drone.location.global_relative_frame.lon
    diff = math.sqrt((hedef.lat - currentLocationLat)**2 + (hedef.lon - currentLocationLon)**2)*1000
    if diff > 0.005:
        print(currentLocation)
        print(diff)
        return True
    else:
        print("Hedefe ulaşıldı")
        return False

def dogrultu_duzelt(drone, aci):
    '''
    Drone'un belirtilen doğrultuya bakmasını sağlar. 0 derece aracı kuzeye doğrultur
    '''
    msg = drone.message_factory.command_long_encode(
        0, 0,       # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0,          #confirmation
        aci,        # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        0,          # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used

    # send command to vehicle
    drone.send_mavlink(msg)

    while abs((drone.attitude.yaw*(180/math.pi)+360)%360 - aci) > 1:
        print("Drone hareket halinde: " + str(abs(drone.attitude.yaw*(180/math.pi))))
        print(aci)
        sleep(0.5)

    print("Dönüş başarılı")

def servo_ayarla(drone, servo, pwm):
    '''
    Verilen PWM değerini belirlenen servoya yazar
    '''
    servo_komut = drone.message_factory.command_long_encode(
    0, 0,           # target system, target component
    mavutil.mavlink.MAV_CMD_DO_SET_SERVO, #command
    0,              #confirmation
    servo,          # param 1, Servo number
    pwm,            # PWM value
    0,              # param 3
    0,              # param 4
    0, 0, 0)        # param 5 ~ 7 not used
    drone.send_mavlink(servo_komut)
