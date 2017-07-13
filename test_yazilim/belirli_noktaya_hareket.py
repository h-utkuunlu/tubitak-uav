#!/usr/bin/python
# -*- coding: UTF-8 -*-

from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from time import sleep, time
import numpy as np
import math
import tanimlar as t

###############################################################################

### Değişken kurulumu

irtifa = 8

## Koordinatlar

#ev_konum = 40.084565, 32.594659
hedef_konum = LocationGlobalRelative(40.085264, 32.593656, irtifa) # Matrisin koordinatlarını gir

## Cihazla iletişim
drone = connect('/dev/ttyUSB0', wait_ready=True, baud=921600) # '/dev/ttyS0'

### Drone Hareketleri
## Kalkış & Ev konumu belirleme
ev_konum = t.hazirlik_ve_kalkis(drone, irtifa)

## Kalkış sonrası hedef noktasına uç

drone.simple_goto(hedef_konum, 10)

while t.hedef_varis(drone, hedef_konum):
    sleep(1)

sleep(10)

## Kalkış konumuna git
drone.mode = VehicleMode("RTL")
while t.hedef_varis(drone, ev_konum):
    sleep(1)

print("Kalkış noktasına ulaşıldı")
sleep(2)
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
