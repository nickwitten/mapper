import os
import pickle as pkl
import numpy as np
import time
import re
import threading

DATA_DIR = "/home/pi/workspace/mapper/plot_data/"
temp_fp = DATA_DIR + "temp.pkl"
fp = DATA_DIR + "points.pkl"
coords = list()
mbed = open("/dev/ttyACM0", "r")
mbed_out = open("/dev/ttyACM0", "w")

def passthrough():
    while True:
        mbed_out.write(input())
        mbed_out.flush()

if __name__ == '__main__':
    t = threading.Thread(target=passthrough)
    t.start()
    while True:
        try:
            new_lines = mbed.readlines()
        except Exception as e:
            print(e)
            print("Error reading from serial device, resetting connection.")
            mbed.close()
            mbed = open("/dev/ttyACM0", "r")
            continue
        for line in new_lines:
            if re.fullmatch(r'^-?[0-9]+, -?[0-9]+\n', line) is not None:
                coords.append(line.split(', '))
            if 'reset' in line:
                coords = list()
                print("Data reset")
        try:
            with open(temp_fp, "wb") as wf:
                pkl.dump(np.array(coords).astype(np.int32), wf)
            os.rename(temp_fp, fp)  # to make the file write atomic
            print(f'{len(coords)} points saved.')
        except Exception as e:
            print(e)
            print("\nCould not write to file")
        time.sleep(0.5)


