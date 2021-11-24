import numpy as np
from matplotlib import pyplot as plt
from scipy.signal import convolve2d

arr = np.zeros((100, 100))


for i in range(0, 50):
    for j in range(0, 100):
        arr[i, j] = i * 0.01

# for i in range(50, 100):
#     for j in range(0, 100):
#         arr[i, j] = np.random.uniform(0, 1)


laplacian = np.array([[0, -1, 0],
                      [-1, 4, -1],
                      [0, -1, 0]])

# laplacian = np.array([[-1, -1, -1],
#                       [-1, 8, -1],
#                       [-1, -1, -1]])

convolutions = convolve2d(arr, laplacian, 'valid')


plt.matshow(convolutions)
plt.colorbar()
plt.show()


