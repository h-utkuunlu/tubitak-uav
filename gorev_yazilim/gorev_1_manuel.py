#!/usr/bin/python
# -*- coding: UTF-8 -*-

from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from time import sleep, time
import numpy as np
import cv2
import imutils
import math
import tanimlar as t

###############################################################################

### Değişken kurulumu

## Başlangıç değerleri
sayac = 0
#hava_hiz = 10

## Hedef tanımları
irtifa = 8          # hesaplama: irtifa = ((matris eni)/2)/(arctan(24.4 derece)) . 5 metre matris eni icin h = 5.51. 6 - 10 arası bir irtifanın iş görmesi gerekir. Matris yüzeyinden baz alarak
matris_konum = LocationGlobalRelative(40.087013, 32.594787, irtifa)
teslim_konum = LocationGlobalRelative(40.084942, 32.593614, irtifa)
dogrultu = 230

### Drone ile iletişim
drone = connect('/dev/ttyUSB0', wait_ready=True)

### Drone Hareketleri

while drone.mode.name != "GUIDED":
    print(drone.mode.name)
    print("Drone otopilotta değil")
    sleep(0.5)

### Drone Hareketleri

## Ev konumu belirleme
ev_konum = drone.location.global_relative_frame
ev_dogrultu = drone.attitude.yaw*(180/math.pi)+360)%360

## Kalkış sonrası matrisin üzerine uç
drone.simple_goto(matris_konum)

while t.hedef_varis(drone, matris_konum):
    sleep(1)

t.dogrultu_duzelt(drone, ev_dogrultu)

# Kamerayı başlat ve görüntüyü işle
kamera = t.WebcamVideoStream().start()

while True:

    # goruntu = cv2.imread('color_grab.png', -1) # Kamera ile değiştirilecek
    goruntu = kamera.read()
    matris = t.goruntu_isleme(goruntu)

    if matris != 'Hata': # 16 kare bulundu
        if matris_eski == matris: # Kareler değişmedi
            print('Kareler değişmedi')
            sleep(0.5)
        else: # Kareler değişti
            print("Kareler belirlendi. Yazılıyor")
            matris_eski = matris
            t.sd_kart(matris)
            sayac += 1
    else:
        print("Hata")
        sleep(0.2)

    if sayac >= 1: # değişecek
        break

### Görev tamam
print('Görev tamam')
kamera.stop()

## Kalkış konumuna git
drone.simple_goto(ev_konum, 10)
while hedef_varis(ev_konum):
    sleep(1)

print("Kalkış noktasına ulaşıldı")
drone.mode = VehicleMode("STABILIZE")

while drone.armed:
    print("İniş bekleniyor")
    sleep(2)

## Cihaz kapatma komutları
print("Kapatılıyor")

cv2.destroyAllWindows()
drone.close()

print("Kapatma başarılı. Program sona erdi")
