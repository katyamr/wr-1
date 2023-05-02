#!/usr/bin/env python3
import serial
import sys
import time

if len(sys.argv) < 2:
    from serial.tools.list_ports import comports
    for p in comports():
        print(f'{p[0]}: {p[1]}')
    exit()

if len(sys.argv) >= 3:
    rate = int(sys.argv[2])
else:
    #rate = 115200
    rate = 9600

def connect():
    with serial.Serial(sys.argv[1], rate, timeout=1) as ser:
        fname = time.strftime('data-%Y%m%d-%H%M%S.txt')
        print(f'[{time.monotonic()}: {fname} {ser.name}]')
        last_time = 0

        with open(fname, "wb") as f:
            while True:
                s = ser.readline()
                if s:
                    t = time.monotonic()
                    if t - last_time > 1:
                        f.write(time.strftime('\n%Y%m%d-%H%M%S\n').encode())
                    last_time = t
                    f.write(s)
                    print(s.decode(), end='')

while True:
    try:
        connect()
    except serial.serialutil.SerialException as e:
        pass
    time.sleep(0.5)
