from rover_common import aiolcm
from rover_msgs import GPS, IMU
from .splines import path_generator
import numpy as np
from sys import stdin


def main():
    x_str = stdin.readline().replace("\n", "")
    x = list(map(float, x_str.split()))
    num_pts = int(stdin.readline().replace("\n", ""))
    x = np.linspace(x[0], x[1], num=num_pts)
    y_str = stdin.readline().replace("\n", "")
    y = list(map(float, y_str.split()))
    y = np.array(y)
    path_data = path_generator(x, y)

    lcm = aiolcm.AsyncLCM()

    for point_info in path_data:
        gps = GPS()
        gps.latitude_deg = point_info.latitude_deg
        gps.longitude_deg = point_info.longitude_deg
        gps.latitude_min = point_info.latitude_min
        gps.longitude_min = point_info.longitude_min
        gps.bearing_deg = point_info.bearing_gps
        gps.speed = point_info.speed
        lcm.publish('/gps', gps.encode())

        imu_msg = IMU()
        imu_msg.accel_x = point_info.accel_x
        imu_msg.accel_y = point_info.accel_y
        imu_msg.accel_z = point_info.accel_z
        imu_msg.gyro_x = point_info.gyro_x
        imu_msg.gyro_y = point_info.gyro_y
        imu_msg.gyro_z = point_info.gyro_z
        imu_msg.mag_x = point_info.magnitude_x
        imu_msg.mag_y = point_info.magnitude_y
        imu_msg.mag_z = point_info.magnitude_z
        imu_msg.bearing = point_info.bearing_imu
        lcm.publish('/imu', imu_msg.encode())

if __name__ == "__main__":
    main()
