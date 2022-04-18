//Use the original mapper position as  [0,0]
import numpy as np
from matplotlib import pyplot as plt

data = np.array([
    [1, 2],
    [2, 3],
    [3, 6],
])
x, y = data.T
plt.scatter(x,y)
plt.show()

