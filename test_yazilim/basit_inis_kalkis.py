#!/usr/bin/python
# -*- coding: UTF-8 -*-

print "Basit iniş kalkış"

from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from time import sleep, time
import numpy as np
import math
import tanimlar as t
### Fonksiyon ve sınıf tanımları

sleep(10)

###############################################################################

### Değişken kurulumu
irtifa = 8

## Cihazla iletişim
drone = connect('/dev/ttyUSB0', wait_ready=True, baud=921600) # '/dev/ttyS0'

### Drone Hareketleri
## Kalkış & Ev konumu belirleme
ev_konum = t.hazirlik_ve_kalkis(drone, irtifa)

print(drone.location.global_relative_frame)
sleep(10)
print(drone.location.global_relative_frame)

## İniş
print("İnişe geçiliyor")
drone.mode = VehicleMode("LAND")
while drone.location.global_relative_frame.alt > 0.08:
    print(drone.location.global_relative_frame.alt)
    sleep(0.5)

print("İniş gerçekleşti")

## Cihaz kapatma komutları
print("Kapatılıyor")
drone.close()

print("Kapatma başarılı. Program sona erdi")
