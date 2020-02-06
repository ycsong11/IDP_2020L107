import csv
import numpy as np
import os
import matplotlib.pyplot as plt

path = os.path.join(
    r"C:\Users\theof\Desktop\IDP\IDP_2020L107\calibration", "data_long.csv")
reading, distance = [], []
with open(path, 'r') as f:
	reader = csv.reader(f)
	for row in reader:
		distance.append(int(row[0]))
		reading.append(int(row[1]))

fit = np.polyfit(reading, distance, 3)
f = np.poly1d(fit)

r_new = np.linspace(100, 500, 100)
d_new = f(r_new)

print(f)

plt.plot(reading, distance, 'o', r_new, d_new, '-')
plt.legend(['data', 'fit'], loc='best')
plt.show()