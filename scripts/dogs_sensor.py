import numpy as np
import matplotlib.pyplot as plt
FILENAME = "data.txt"
data = np.loadtxt(FILENAME)
print(data.size)
plt.plot(data)
plt.show()
