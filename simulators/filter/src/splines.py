import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate
from math import atan, pi


def within_five(x_current_diff, x_prev_diff):
    upper_bound = x_prev_diff * 1.05
    lower_bound = x_prev_diff * 0.95
    if (x_current_diff > upper_bound) or (x_current_diff < lower_bound):
        return False
    return True


def represent_data(x, y):
    #24 data points first row
    # Check if x and y are same
    if len(x) != len(y):
        print("Error: arrays are of different sizes.")
        exit(1)

    tck = interpolate.splrep(x, y, s=100)
    x_start = x[0]
    x_end = x[len(x) - 1]
    x_path = np.linspace(x_start,x_end, num=100)
    y_path = interpolate.splev(x_path, tck, der=0)

    vel_direcs = []
    accel_direcs = []
    d_x = (x_end - x_start) / (len(x_path) - 1)

    x_prev_diff = x_path[1] - x_path[0]
    for i in range(0, len(x_path) - 1):
        # Check if x is evenly spaced
        x_current_diff = x_path[i + 1] - x_path[i]
        if not within_five(x_current_diff, x_prev_diff):
            print("Error: x values are not evenly spaced.")
            exit(1)
        x_prev_diff = x_current_diff
        d_y = y_path[i+1] - y_path[i]
        if d_y < 0:
            vel_direcs.append(abs(atan(d_y / d_x) - (pi/2)))
        elif d_y > 0:
            vel_direcs.append(atan(d_x / d_y))
        else:
            vel_direcs.append(pi/2)

    vel_direcs_np = np.array(vel_direcs)

    x_vel_mag = np.diff(x_path)
    y_vel_mag = np.diff(y_path)

    x_acc_mag = np.diff(x_vel_mag)
    y_acc_mag = np.diff(y_vel_mag)

    for y_acc in y_acc_mag:
        if y_acc > 0:
            accel_direcs.append(0)
        elif y_acc < 0:
            accel_direcs.append(pi)
        else:
            accel_direcs.append(pi/2)

    accel_direcs_np = np.array(accel_direcs)

    print(len(x_path))
    print(len(y_path))
    print(len(vel_direcs_np))
    print(len(accel_direcs_np))

    plt.figure(1)
    plt.subplot(511)
    plt.plot(x, y, 'x', x_path, y_path,'b')
    plt.legend(['Data', 'Cubic Spline'])
    plt.title('Cubic-spline interpolation')
    plt.subplot(512)
    plt.plot(x_path[:-1], x_vel_mag, 'k', x_path[:-1], y_vel_mag, 'b')
    plt.title('Velocity Magnitudes')
    plt.legend(['X Vel', 'Y Vel'])
    plt.subplot(513)
    plt.plot(x_path[:-2], x_acc_mag, 'k', x_path[:-2], y_acc_mag, 'b')
    plt.title('Acceleration Magnitudes')
    plt.legend(['X Acc', 'Y Acc'])
    plt.subplot(514)
    plt.plot(x_path[:-1], vel_direcs_np, 'b')
    plt.title('Velocity Bearing')
    plt.legend(['Bearing Angle (rad)'], loc = 'upper left')
    plt.subplot(515)
    plt.plot(x_path[:-2], accel_direcs_np,'b')
    plt.title('Acceleration Bearing')
    plt.legend(['Bearing Angle (rad)'], loc = 'upper left')
    plt.show()
