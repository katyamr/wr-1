#!/usr/bin/env python
import serial
import sys
import time

rate = 115200

def connect():
    with serial.Serial(sys.argv[1], rate, timeout=1) as ser:
        print(f'[{time.monotonic()}: {ser.name}]')

        while True:
            s = ser.readline()
            if s:
                s = s.decode()
                print(s, end='')

while True:
    try:
        connect()
    except serial.serialutil.SerialException as e:
        time.sleep(0.1)
