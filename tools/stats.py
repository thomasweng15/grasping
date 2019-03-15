import csv
import numpy as np
import matplotlib.pyplot as plt

q = []
with open('gqcnn-output/out.csv', 'r') as f:
    reader = csv.reader(f)
    for row in reader:
        if row[1] != '':
            q.append(float(row[1]))
q_mean = np.mean(q)

q_fixed = []
with open('gqcnn-output-fixed-tblheight/out.csv', 'r') as f:
    reader = csv.reader(f)
    for row in reader:
        if row[1] !='':
            q_fixed.append(float(row[1]))
q_fixed_mean = np.mean(q_fixed)

ax1=plt.subplot(221)
n1, _, _ = ax1.hist(q, range=(0.0, 1.0), bins=100)
ax1.set_title('no height adjustment')

ax2=plt.subplot(222)
n2, _, _ = ax2.hist(q_fixed, range=(0.0, 1.0), bins=100)
ax2.set_title('fixed height adjustment')

# Set y limit
ymax = np.max([n1.max(), n2.max()]) + 0.5
ax1.set_ylim((0, ymax))
ax2.set_ylim((0, ymax))

# Show mean
ax1.vlines([q_mean], 0, ymax, colors='r')
ax2.vlines([q_fixed_mean], 0, ymax, colors='r')

plt.show()
