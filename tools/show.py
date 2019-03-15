import sys
import numpy as np
import matplotlib.pyplot as plt

im = np.load(sys.argv[1])
print("Max: %f", np.max(im))
plt.imshow(im, cmap=plt.cm.gray_r)
plt.show()