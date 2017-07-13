#!/usr/bin/python
# -*- coding: UTF-8 -*-

print "Kamera test"

from __future__ import print_function
from threading import Thread
from shape_detector import ShapeDetector
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from time import sleep, time
import numpy as np
import cv2
import imutils
import math
import picamera
import tanimlar as t
import datetime

###############################################################################

sleep(10)

# Kamerayı başlat ve görüntüyü işle
kamera = picamera.PiCamera()

for i in range(3):

    dosya_adi = datetime.datetime.fromtimestamp(time()).strftime('%Y-%m-%d %H-%M-%S') + '.jpg'
    kamera.capture(dosya_adi)
    goruntu = cv2.imread(dosya_adi, -1)

    matris = t.goruntu_isleme(goruntu)

    if matris != 'Hata': # 16 kare bulundu
        if matris_eski == matris: # Kareler değişmedi
            print('Kareler değişmedi')
            sleep(3)
        else: # Kareler değişti
            print("Kareler belirlendi. Yazılıyor")
            matris_eski = matris
            t.sd_kart(matris)

    else:
        print("Hata")
        sleep(3)

### Görev tamam
print('Görev tamam')
kamera.close()

## Cihaz kapatma komutları
print("Kapatılıyor")

cv2.destroyAllWindows()

print("Kapatma başarılı. Program sona erdi")
