#!/usr/bin/python
# -*- coding: UTF-8 -*-
print "Kumanda kontrol"

from __future__ import print_function
from threading import Thread
from shape_detector import ShapeDetector
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from time import sleep, time
import numpy as np
import cv2
import imutils
import math
import tanimlar as t

###############################################################################

### Değişken kurulumu

irtifa = 8

sleep(10)

## Cihazla iletişim
drone = connect('/dev/ttyUSB0', wait_ready=True, baud=921600)

curr_time = time()

while drone.mode.name != "GUIDED":
    print(drone.mode.name)
    print("Drone otopilotta değil")
    sleep(0.5)
    if time() - curr_time > 10:
        drone.mode = VehicleMode("GUIDED")

### Drone Hareketleri

## Kalkış & Ev konumu belirleme
ev_konum = t.hazirlik_ve_kalkis(drone, irtifa)

print('Kalkış başarılı')
sleep(5)
print('İnişe geçiliyor')

## İniş
drone.mode = VehicleMode("LAND")
while drone.location.global_relative_frame.alt > 0.25:
    print(drone.location.global_relative_frame.alt)
    sleep(0.5)

print("İniş gerçekleşti")

## Cihaz kapatma komutları
print("Kapatılıyor")

drone.close()

print("Kapatma başarılı. Program sona erdi")
