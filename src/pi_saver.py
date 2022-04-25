import pickle as pkl
import numpy as np
import time
import re

DATA_DIR = "/home/pi/workspace/4180_team_project_2D_mapping_bot/plot_data/"
coords = list()
with open("/dev/ttyACM0") as f:
    while True:
        new_lines = f.readlines()
        for line in new_lines:
            if re.fullmatch(r'^-?[0-9]+, -?[0-9]+\n', line) is not None:
                coords.append(line.split(', '))
            if 'reset' in line:
                coords = list()
                print("Data reset")
        try:
            with open(DATA_DIR + "points.pkl", "wb") as wf:
                pkl.dump(np.array(coords).astype(np.int32), wf)
                print(f'{len(coords)} points saved.')
        except Exception as e:
            print(e)
            print("\nCould not write to file")
        time.sleep(0.5)


