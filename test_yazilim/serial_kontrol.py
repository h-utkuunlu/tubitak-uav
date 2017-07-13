#!/usr/bin/python
# -*- coding: UTF-8 -*-

from __future__ import print_function
from time import sleep, time
import tanimlar as t

### Değişken kurulumu

print("Serial bağlantı kontrol yazılımı")

string_set = ['1002201201120021', '1120221202102210', '2002102100122021']

for i in range(1, 4):
    print("Örüntü " + str(i) + " yazılıyor"
    t.sd_kart(string_set[i-1].encode())
    sleep(5)
