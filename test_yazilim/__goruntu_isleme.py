#!/usr/bin/python
# -*- coding: UTF-8 -*-

from __future__ import print_function
from threading import Thread
from shape_detector import ShapeDetector
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from time import sleep, time
import numpy as np
import cv2
import imutils
import math

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

def goruntu_isleme(grt):
    """
    Kameradan sağlanan görüntüyü (grt) inceleyerek matris renklerini belirler. Renklerin baş harflerini dizi olarak gönderir. Renk sınırları fonksiyon içinde değiştirilmelidir.
    """

    hsv = cv2.cvtColor(grt, cv2.COLOR_BGR2HSV)
    rect_count = 0
    merkezler = []

    # Renk sınırları: (renk kodu, (hsv alt sınır, hsv üst sınır))
    renk_sinir = [
    ('k', (np.array([0, 50, 50], dtype = np.uint8) , np.array([12, 255, 255], dtype = np.uint8))), # Kırmızı
#    ('k', (np.array([170, 50, 50], dtype = np.uint8) , np.array([180, 255, 255], dtype = np.uint8))),
    ('m', (np.array([97, 80, 50], dtype = np.uint8) , np.array([105, 255, 255], dtype = np.uint8))), # Mavi
    ('s', (np.array([15, 175, 50], dtype = np.uint8) , np.array([30, 255, 255], dtype = np.uint8))), # Sarı
    ]

    for (renk, (alt, ust)) in renk_sinir:

        # Verilen rengi algıla ve şekilleri birleştir

        mask = cv2.inRange(hsv, alt, ust)

        cv2.imshow("Image", mask)
        cv2.waitKey(0)

        blurred = cv2.GaussianBlur(mask, (15, 15), 0)

        cv2.imshow("Image", blurred)
        cv2.waitKey(0)

        thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]

        cv2.imshow("Image", thresh)
        cv2.waitKey(0)

        erndi = cv2.erode(thresh, None, iterations=2)
        erndi = cv2.dilate(erndi, None, iterations=2)

        cv2.imshow("Image", erndi)
        cv2.waitKey(0)

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

    cv2.imshow("Image", grt)
    cv2.waitKey(0)

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

    # renk_sira = [renk_matrisi[1][0],
    ### Buradaki sistemi henüz bilmiyorum. Ondan şimdilik boş

    # return sd_kod
    print(renk_str)

###############################################################################

### Değişken kurulumu

## Başlangıç değerleri
matris_eski = None
sayac = 0

# Kamerayı başlat ve görüntüyü işle
kamera = WebcamVideoStream().start()

while True:

    # goruntu = cv2.imread('color_grab.png', -1) # Kamera ile değiştirilecek
    goruntu = kamera.read()
    matris = goruntu_isleme(goruntu)

    if matris != 'Hata': # 16 kare bulundu
        if matris_eski == matris: # Kareler değişmedi
            print('Kareler değişmedi')
            sleep(0.5)
        else: # Kareler değişti
            print("Kareler belirlendi. Yazılıyor")
            matris_eski = matris
            sd_kart(matris)
            sayac += 1
    else:
        print("Hata")
        sleep(0.2)

### Görev tamam
print('Görev tamam')

kamera.stop()

cv2.destroyAllWindows()

print("Kapatma başarılı. Program sona erdi")
