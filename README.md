# ESP32 Weather Station
This is a part of ESP32 Weather Station project that contains firmware for ESP32.

## Companion blog post
The inner workings of the source code is throughly explained in a companion blog post.

## Installation
```
export IDF_PATH=/path/to/esp-idf-framework
idf.py build

# For Mac you can use /dev/cu.SLAB_USBtoUART
idf.py -p [device] flash
idf.py -p [device] monitor
```