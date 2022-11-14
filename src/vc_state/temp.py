import numpy as np
import matplotlib.pyplot as plt

def distances(x):
    d = np.zeros((len(x), len(x)))
    for i in range(0, len(x)):
        for j in range(0, i):
            d[i,j] = np.sqrt((x[i] - x[j])**2)
    return d

x = np.random.randn(4)*10
x = x.reshape((len(x), 1))
d = distances(x)
print(d)
print(np.max(d))
print(x)
# plot x
plt.plot(x, np.zeros_like(x), 'o')
# plt.imshow(d)
plt.show()