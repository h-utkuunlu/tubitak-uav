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

## Kalkış & Ev konumu belirleme
(ev_konum, ev_dogrultu) = t.hazirlik_ve_kalkis(drone, irtifa)

## Kalkış sonrası matrisin üzerine uç
drone.simple_goto(matris_konum)

while t.hedef_varis(drone, matris_konum):
    sleep(1)

t.dogrultu_duzelt(drone, ev_dogrultu)

# Kamerayı başlat ve görüntüyü işle
kamera = t.WebcamVideoStream().start()

yeni_set = True
gorev_baslangic = time()

while True:

    # goruntu = cv2.imread('color_grab.png', -1) # Kamera ile değiştirilecek
    goruntu = kamera.read()
    matris = t.goruntu_isleme(goruntu)

    if matris != 'Hata': # 16 kare bulundu

        if (matris == 'k'*16 or matris == 's'*16 or matris == 'm'*16):
            print("Kareler değişim düzeninde")
            yeni_set = True
            sleep(2)
        elif yeni_set: #
            print("Kareler belirlendi. Yazılıyor")
            yeni_set = False
            t.sd_kart(matris)
            sayac += 1
            sleep(2)
        else:
            print("Kareler henüz değişmedi")
            sleep(1)
    else:
        sleep(0.05)

    if sayac >= 3: # değişecek
        break

    if gorev_baslangic - time() > 240: # Gorev baslayali 4 dakika oldu. Bir sorun var demektir. Drone'u geri cagir
        break

### Görev tamam
print('Görev tamam')
kamera.stop()

## Kalkış konumuna git
drone.mode = VehicleMode("RTL")
while t.hedef_varis(drone, ev_konum):
    sleep(1)

print("Kalkış noktasına ulaşıldı")

## İniş
drone.mode = VehicleMode("LAND")
while drone.location.global_relative_frame.alt > 0.25:
    print(drone.location.global_relative_frame.alt)
    sleep(0.5)

print("İniş gerçekleşti")

## Cihaz kapatma komutları
print("Kapatılıyor")

cv2.destroyAllWindows()
drone.close()

print("Kapatma başarılı. Program sona erdi")
