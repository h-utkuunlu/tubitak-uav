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
kirmizi_paket = 6
mavi_paket = 7
sari_paket = 8

servo_acik = 1000
servo_kapali = 1800

## Cihazla iletişim
drone = connect('/dev/ttyACM0', wait_ready=True)
### Drone Hareketleri

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
cv2.destroyAllWindows()
drone.close()

print("Kapatma başarılı. Program sona erdi")
