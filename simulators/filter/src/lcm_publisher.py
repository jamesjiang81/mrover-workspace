from rover_common import aiolcm
from rover_msgs import GPS, IMU

lcm = aiolcm.AsyncLCM()

gps = GPS()
gps.latitude_deg = int(latitude/100)
gps.longitude_deg = int(longitude/100)
gps.latitude_min = latitude - (gps.latitude_deg * 100)
gps.longitude_min = longitude - (gps.longitude_deg * 100)
gps.bearing_deg = bearing
gps.speed = speed
lcm.publish('/gps', gps.encode())


imu_msg = IMU()
imu_msg.accel_x = accel_x
imu_msg.accel_y = accel_y
imu_msg.accel_z = accel_z
imu_msg.gyro_x = gyro_x
imu_msg.gyro_y = gyro_y
imu_msg.gyro_z = gyro_z
imu_msg.mag_x = mag_x
imu_msg.mag_y = mag_y
imu_msg.mag_z = mag_z
imu_msg.bearing = (angle_z - (drift * num_readings)) % 360
lcm.publish('/imu', imu_msg.encode())
