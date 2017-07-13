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
irtifa = 8          # hesaplama: irtifa = ((matris eni)/2)/(arctan(24.4 derece)) . 5 metre matris eni icin h = 5.51. 6 - 10 arası bir irtifanın iş görmesi gerekir. Matris yüzeyinden baz alarak

matris_konum = LocationGlobalRelative(40.087013, 32.594787, irtifa)
teslim_konum = LocationGlobalRelative(40.084942, 32.593614, irtifa)
dogrultu = 230
#ev_konum = 40.084565, 32.594659

servo_kapali = 1000
servo_acik = 1800

## Drone bağlantı
drone = connect('/dev/ttyUSB0', wait_ready=True) # '/dev/ttyS0'

### Drone Hareketleri

# Servo konumlari
kirmizi_paket = 6
mavi_paket = 7
sari_paket = 8

# servo_ayarla(drone, kirmizi_paket, t.servo_kapali) # Muhtemelen calismayacak
# servo_ayarla(drone, mavi_paket, t.servo_kapali)
# servo_ayarla(drone, sari_paket, t.servo_kapali)

while drone.mode.name != "GUIDED":
    print(drone.mode.name)
    print("Drone otopilotta değil")
    sleep(0.5)

## Ev konumu belirleme
ev_konum = drone.location.global_relative_frame

## Kalkış sonrası matrisin üzerine uç
drone.simple_goto(matris_konum)

while t.hedef_varis(drone, matris_konum):
    sleep(1)

t.dogrultu_duzelt(drone, dogrultu)

# Kamerayı başlat ve görüntüyü işle
kamera = t.WebcamVideoStream().start()

while True:

    # goruntu = cv2.imread('color_grab.png', -1) # Kamera ile değiştirilecek
    goruntu = kamera.read()
    matris = t.goruntu_isleme(goruntu)

    if matris != 'Hata': # 16 kare bulundu
        paket_sira = matris[0] + matris[4] + matris[8]
        break
    else:
        print("Hata")
        sleep(0.2)

print('Matris görüntü işleme tamamlandı')
kamera.stop()

## Paraşüt bırakma alanına uç
drone.simple_goto(teslim_konum)

for sira in renk_sira:
    if sira == 'k':
        paket = kirmizi_paket
    elif sira == 's':
        paket = sari_paket
    elif sira == 'm':
        paket = mavi_paket
    print("Bırakılan paket: " + sira)
    servo_komut(drone, paket, t.servo_acik) # Servo değerini belirle!!
    sleep(7)
    servo_komut(drone, paket, t.servo_kapali)
    sleep(1)

print('Paketler bırakıldı')

### Görev tamam
print('Görev tamam')

## Kalkış konumuna git
drone.simple_goto(ev_konum, 10)
while hedef_varis(ev_konum):
    sleep(1)

drone.mode = VehicleMode("STABILIZE")

while drone.armed:
    print("İniş bekleniyor")
    sleep(2)

print("İniş gerçekleşti")

## Cihaz kapatma komutları
print("Kapatılıyor")

cv2.destroyAllWindows()
drone.close()

print("Kapatma başarılı. Program sona erdi")
