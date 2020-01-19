import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate

#24 data points first row
x = np.linspace(42.277,42.292, num=40)
y = np.array([83.737,83.739,83.739,83.740,83.738,83.736,83.736,83.735,83.735,83.732,83.733,83.731,83.730,83.725,83.724,83.723,83.724,83.721,83.723,83.727,83.728,83.733,83.732,83.733,
83.735, 83.737,83.740, 83.742, 83.741, 83.744, 83.745, 83.747, 83.750, 83.753, 83.751, 83.749, 83.748, 83.747,83.748,83.748])
tck = interpolate.splrep(x, y, s=100)
x_path = np.linspace(42.277,42.292, num=100)
y_path = interpolate.splev(x_path, tck, der=0)

x_vel_mag = np.diff(x_path)
y_vel_mag = np.diff(y_path)

x_acc_mag = np.diff(x_vel_mag)
y_acc_mag = np.diff(y_vel_mag)

plt.figure(1)
plt.subplot(311)
plt.plot(x, y, 'x', x_path, y_path,'b')
plt.legend(['Data', 'Cubic Spline'])
plt.title('Cubic-spline interpolation')
plt.subplot(312)
plt.plot(x_path[:-1], x_vel_mag, 'k', x_path[:-1], y_vel_mag, 'b')
plt.title('Velocity Magnitudes')
plt.legend(['X Vel', 'Y Vel'])
plt.subplot(313)
plt.plot(x_path[:-2], x_acc_mag, 'k', x_path[:-2], y_acc_mag, 'b')
plt.title('Acceleration Magnitudes')
plt.legend(['X Acc', 'Y Acc'])
plt.show()