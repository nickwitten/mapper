import matplotlib.pyplot as plt
import numpy as np
import pickle as pkl
import subprocess
import sys


PC_PROJECT_DIR = "C:/Users/nwitt/workspace/mapper/"
PI_PROJECT_DIR = "/home/pi/workspace/mapper/"
DATA_DIR = "plot_data/"
DATA_NAME = "points.pkl"
PI_IP = "128.61.66.118"

while True:
    ret = subprocess.run(["scp", f'pi@{PI_IP}:{PI_PROJECT_DIR}{DATA_DIR}{DATA_NAME}', f'{PC_PROJECT_DIR}{DATA_DIR}{DATA_NAME}'], capture_output=True)
    if ret.returncode != 0:
        print(ret.stderr.decode('utf-8'))
        print(f'Could not reach the data on {PI_IP}')
        continue

    with open(PC_PROJECT_DIR + DATA_DIR + "points.pkl", "rb") as f:
        points = pkl.load(f)

    plt.clf()
    maxes = np.max(points, axis=0)
    xlim = (maxes[0] // 1000) * 1000 + 1000
    ylim = (maxes[1] // 1000) * 1000 + 1000
    plt.gca().set_xlim(-xlim, xlim)
    plt.gca().set_ylim(-ylim, ylim)
    plt.scatter(points.T[0], points.T[1])
    plt.pause(0.05)

