from rover_common import aiolcm
from rover_msgs import GPS, IMU
from splines import path_generator
import numpy as np


def main():
    x = np.linspace(42.277, 42.292, num=40)
    y = np.array(
        [83.737, 83.739, 83.739, 83.740, 83.738, 83.736, 83.736, 83.735, 83.735, 83.732, 83.733, 83.731, 83.730, 83.725,
         83.724, 83.723, 83.724, 83.721, 83.723, 83.727, 83.728, 83.733, 83.732, 83.733,
         83.735, 83.737, 83.740, 83.742, 83.741, 83.744, 83.745, 83.747, 83.750, 83.753, 83.751, 83.749, 83.748, 83.747,
         83.748, 83.748])
    path_data = path_generator(x, y)

    lcm = aiolcm.AsyncLCM()

    for point_info in path_data:
        gps = GPS()
        gps.latitude_deg = float(point_info.latitude_deg)
        gps.longitude_deg = float(point_info.longitude_deg)
        gps.latitude_min = float(point_info.latitude_min)
        gps.longitude_min = float(point_info.longitude_min)
        gps.bearing_deg = float(point_info.bearing_gps)
        gps.speed = float(point_info.speed)
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
