import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate
from math import atan, pi, sqrt


# returns the average over a list
def average_diff(x_path):
    total_diff = 0
    for i in range(1, len(x_path)):
        total_diff += x_path[i] - x_path[i - 1]
    return total_diff / len(x_path)


class GeneratedData:
    def __init__(self):
        self.latitude_deg = 0
        self.longitude_deg = 0
        self.latitude_min = 0
        self.longitude_min = 0
        self.bearing_gps = 0
        self.speed = 0

        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0
        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0
        self.magnitude_x = 0.0
        self.magnitude_y = 0.0
        self.magnitude_z = 0.0
        self.bearing_imu = 0.0

    def set_values(self, latitude_deg, latitude_min, longitude_deg, longitude_min, bearing_gps, speed, accel_x, accel_y, magnitude_x, magnitude_y):
        self.latitude_deg = latitude_deg
        self.latitude_min = latitude_min
        self.longitude_deg = longitude_deg
        self.longitude_min = longitude_min
        self.bearing_gps = bearing_gps
        self.speed = speed
        self.accel_x = accel_x
        self.accel_y = accel_y
        self.magnitude_x = magnitude_x
        self.magnitude_y = magnitude_y


# Checks if the x_current_diff is within 5% of x_prev_diff
def within_five(x_current_diff, avg_diff):
    upper_bound = avg_diff * 1.05
    lower_bound = avg_diff * 0.95
    return lower_bound <= x_current_diff <= upper_bound


def generate_path_interpolation(x, y):
    x_start = x[0]
    x_end = x[len(x) - 1]
    tck = interpolate.splrep(x, y, s=100)
    x_path = np.linspace(x_start, x_end, num=100)
    y_path = interpolate.splev(x_path, tck, der=0)
    return x_path, y_path


def calculate_vel_direc(gps_path, x_start, x_end):
    vel_direcs = np.array([])
    # d_x is the average difference between x values
    d_x = (x_end - x_start) / (len(gps_path[0]) - 1)
    # x_avg is the average x coordinate, this is used to check that each pairs of adjacent x values do not
    # differ significantly
    x_diff_avg = average_diff(gps_path[0])

    for i in range(0, len(gps_path[0]) - 1):
        # Check if x is evenly spaced
        x_current_diff = gps_path[0][i + 1] - gps_path[0][i]
        if not within_five(x_current_diff, x_diff_avg):
            print("Error: x values are not evenly spaced.")
            exit(1)
        d_y = gps_path[1][i + 1] - gps_path[1][i]
        if d_y < 0:
            vel_direcs = np.append(vel_direcs, abs(atan(d_y / d_x) - (pi / 2)))
        elif d_y > 0:
            vel_direcs = np.append(vel_direcs, atan(d_x / d_y))
        else:
            vel_direcs = np.append(vel_direcs, pi / 2)

    return vel_direcs


# returns a tuple where first index is an np array of x velocity magnitudes, second index is np array of
# y velocity magnitudes
def calculate_vel_mag(gps_path):
    x_vel_mag = np.diff(gps_path[0])
    y_vel_mag = np.diff(gps_path[1])
    return x_vel_mag, y_vel_mag


# returns a tuple where the first index is an np array of x acceleration magnitudes, second index is np array of
# y acceleration magnitudes
def calculate_accel_mag(path_vel_mag):
    x_acc_mag = np.diff(path_vel_mag[0])
    y_acc_mag = np.diff(path_vel_mag[1])
    return x_acc_mag, y_acc_mag


# returns an np array of acceleration directions
def calculate_accel_direc(path_accel_mag):
    accel_direcs = np.array([])
    for y_acc in path_accel_mag[1]:
        if y_acc > 0:
            accel_direcs = np.append(accel_direcs, 0)
        elif y_acc < 0:
            accel_direcs = np.append(accel_direcs, pi)
        else:
            accel_direcs = np.append(accel_direcs, pi / 2)

    return accel_direcs


def plot_x_y_coordinates(x, y, gps_path):
    plt.figure(1)
    plt.subplot(511)
    plt.plot(x, y, 'x', gps_path[0], gps_path[1], 'b')
    plt.legend(['Data', 'Cubic Spline'])
    plt.title('Cubic-spline interpolation')


def plot_velocity_magnitudes(gps_path, path_vel_mag):
    plt.subplot(512)
    plt.plot(gps_path[0][:-1], path_vel_mag[0], 'k', gps_path[0][:-1], path_vel_mag[1], 'b')
    plt.title('Velocity Magnitudes')
    plt.legend(['X Vel', 'Y Vel'])


def plot_acceleration_magnitudes(gps_path, path_accel_mag):
    plt.subplot(513)
    plt.plot(gps_path[0][:-2], path_accel_mag[0], 'k', gps_path[0][:-2], path_accel_mag[1], 'b')
    plt.title('Acceleration Magnitudes')
    plt.legend(['X Acc', 'Y Acc'])


def plot_velocity_bearings(gps_path, path_vel_direc):
    plt.subplot(514)
    plt.plot(gps_path[0][:-1], path_vel_direc, 'b')
    plt.title('Velocity Bearing')
    plt.legend(['Bearing Angle (rad)'], loc = 'upper left')


def plot_acceleration_bearings(gps_path, path_accel_direc):
    plt.subplot(515)
    plt.plot(gps_path[0][:-2], path_accel_direc, 'b')
    plt.title('Acceleration Bearing')
    plt.legend(['Bearing Angle (rad)'], loc='upper left')
    plt.show()


def publish_data(gps_path, path_vel_direc, path_vel_mag, path_accel_mag):
    chinese = []
    for i in range(0, len(path_vel_mag[0]) - 1):
        gen_data = GeneratedData()
        long_deg = int(gps_path[0][i])
        long_min = (gps_path[0][i] - long_deg) * 60
        lat_deg = int(gps_path[1][i])
        lat_min = (gps_path[1][i] - lat_deg) * 60
        bearing = path_vel_direc[i]
        speed = calculate_speed(path_vel_mag[0][i], path_vel_mag[1][i])
        x_accel = 0.0
        y_accel = path_accel_mag[1][i]
        # magnitude_x and magnitude_y are both 0 (last 2 args) because we didn't know what to do for that
        gen_data.set_values(lat_deg, lat_min, long_deg, long_min, bearing, speed, x_accel, y_accel, 0, 0)
        chinese.append(gen_data)
    return chinese


def calculate_speed(x_vel_mag, y_vel_mag):
    return sqrt(x_vel_mag ** 2 + y_vel_mag ** 2)


def path_generator(x, y):
    # 24 data points first row
    # Check if x and y are same
    if len(x) != len(y):
        print("Error: arrays are of different sizes.")
        exit(1)

    gps_path = generate_path_interpolation(x, y)
    x_start = x[0]
    x_end = x[len(x) - 1]
    path_vel_direc = calculate_vel_direc(gps_path, x_start, x_end)
    path_vel_mag = calculate_vel_mag(gps_path)
    path_accel_mag = calculate_accel_mag(path_vel_mag)
    path_accel_direc = calculate_accel_direc(path_accel_mag)

    plot_x_y_coordinates(x, y, gps_path)
    plot_velocity_magnitudes(gps_path, path_vel_mag)
    plot_acceleration_magnitudes(gps_path, path_accel_mag)
    plot_velocity_bearings(gps_path, path_vel_direc)
    plot_acceleration_bearings(gps_path, path_accel_direc)

    return publish_data(gps_path, path_vel_direc, path_vel_mag, path_accel_mag)


def main():
    x = np.linspace(42.277, 42.292, num=40)
    y = np.array(
        [83.737, 83.739, 83.739, 83.740, 83.738, 83.736, 83.736, 83.735, 83.735, 83.732, 83.733, 83.731, 83.730, 83.725,
         83.724, 83.723, 83.724, 83.721, 83.723, 83.727, 83.728, 83.733, 83.732, 83.733,
         83.735, 83.737, 83.740, 83.742, 83.741, 83.744, 83.745, 83.747, 83.750, 83.753, 83.751, 83.749, 83.748, 83.747,
         83.748, 83.748])
    print(path_generator(x, y))


if __name__ == "__main__":
    main()
