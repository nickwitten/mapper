import pickle as pkl
import numpy as np

coords = list()
with open("/dev/ttyACM0") as f:
# with open("temp.txt") as f:
    while True:
        new_lines = f.readlines()
        new_coords = [line.split(', ') for line in new_lines]
        coords += new_coords
        with open("temp.pkl", "wb") as wf:
            pkl.dump(np.array(coords).astype(np.int32), wf)


