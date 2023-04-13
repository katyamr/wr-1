#!/usr/bin/env python
import sys
import time

start_print_ms = 0
start_print = time.monotonic()

with open(sys.argv[1], 'rb') as f:
    for s in f:
        s = s.decode().strip()
        try:
            a = s.split(';', 2)
            if a[0] == 'RM':
                record_time = int(a[1])
            else:
                record_time = 0
        except:
            record_time = 0

        if record_time > 0:
            now = time.monotonic()
            sleep_sec = (record_time - start_print_ms) / 1000 - (now - start_print)
            # print(f'[%g]' % sleep_sec);
            time.sleep(sleep_sec)

        print(s)
