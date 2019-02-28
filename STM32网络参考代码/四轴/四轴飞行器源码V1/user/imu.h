#ifndef __imu_H
#define __imu_H

void get_euler_angle(float gx, float gy, float gz, float ax, float ay, float az, float *pitch, float *roll);
#endif
