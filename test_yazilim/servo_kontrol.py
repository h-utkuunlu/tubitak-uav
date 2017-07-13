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
import picamera
import datetime
import tanimlar as t

### Değişken kurulumu

# Servo konumlari
kirmizi_paket = 1
mavi_paket = 2
sari_paket = 3

servo_acik = t.servo_acik
servo_kapali = t.servo_kapali

## Cihazla iletişim
drone = connect('/dev/ttyUSB0', wait_ready=True, baud=921600)
### Drone Hareketleri

ev_konum = t.hazirlik_ve_kalkis(2)

t.servo_ayarla(drone, kirmizi_paket, servo_acik)
sleep(1)
t.servo_ayarla(drone, mavi_paket, servo_acik)
sleep(1)
t.servo_ayarla(drone, sari_paket, servo_acik)
sleep(1)
t.servo_ayarla(drone, mavi_paket, servo_kapali)
sleep(1)
t.servo_ayarla(drone, kirmizi_paket, servo_kapali)
sleep(1)
t.servo_ayarla(drone, sari_paket, servo_kapali)
sleep(1)
t.servo_ayarla(drone, mavi_paket, servo_acik)
sleep(1)

## İniş
drone.mode = VehicleMode("LAND")
while drone.location.global_relative_frame.alt > 0.25:
    print(drone.location.global_relative_frame.alt)
    sleep(0.5)

print("İniş gerçekleşti")

cv2.destroyAllWindows()
drone.close()

print("Kapatma başarılı. Program sona erdi")
