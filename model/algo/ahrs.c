/**
 * [H7-015] 迁移: AHRS胶水层
 *   - F4使用预编译 AHRS.lib (M4,不兼容M7)
 *   - 本文件用 MahonyAHRS.c 源码替代 AHRS.lib
 *   - 实现 AHRS_init / AHRS_update / get_angle 等API
 */
#include "AHRS.h"
#include "MahonyAHRS.h"
#include "AHRS_MiddleWare.h"
#include <math.h>

void AHRS_init(fp32 quat[4], const fp32 accel[3], const fp32 mag[3])
{
    fp32 ax = accel[0], ay = accel[1], az = accel[2];
    fp32 mx = mag[0], my = mag[1], mz = mag[2];

    fp32 roll  = atan2f(ay, az);
    fp32 pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
    fp32 yaw;

    fp32 cr = cosf(roll),  sr = sinf(roll);
    fp32 cp = cosf(pitch), sp = sinf(pitch);

    fp32 mag_x = mx * cp + my * sr * sp + mz * cr * sp;
    fp32 mag_y = my * cr - mz * sr;
    yaw = atan2f(-mag_y, mag_x);

    fp32 cy = cosf(yaw * 0.5f), sy = sinf(yaw * 0.5f);
    fp32 cp2 = cosf(pitch * 0.5f), sp2 = sinf(pitch * 0.5f);
    fp32 cr2 = cosf(roll * 0.5f),  sr2 = sinf(roll * 0.5f);

    quat[0] = cy * cp2 * cr2 + sy * sp2 * sr2;
    quat[1] = sy * cp2 * cr2 - cy * sp2 * sr2;
    quat[2] = cy * sp2 * cr2 + sy * cp2 * sr2;
    quat[3] = cy * cp2 * sr2 - sy * sp2 * cr2;
}

bool_t AHRS_update(fp32 quat[4], const fp32 timing_time,
                    const fp32 gyro[3], const fp32 accel[3], const fp32 mag[3])
{
    (void)timing_time;
    MahonyAHRSupdate(quat,
        gyro[0], gyro[1], gyro[2],
        accel[0], accel[1], accel[2],
        mag[0], mag[1], mag[2]);
    return (bool_t)1;
}

fp32 get_yaw(const fp32 quat[4])
{
    return atan2f(2.0f * (quat[0] * quat[3] + quat[1] * quat[2]),
                  1.0f - 2.0f * (quat[2] * quat[2] + quat[3] * quat[3]));
}

fp32 get_pitch(const fp32 quat[4])
{
    fp32 v = 2.0f * (quat[0] * quat[1] - quat[2] * quat[3]);
    if (v > 1.0f)  v = 1.0f;
    if (v < -1.0f) v = -1.0f;
    return asinf(v);
}

fp32 get_roll(const fp32 quat[4])
{
    return atan2f(2.0f * (quat[0] * quat[2] + quat[1] * quat[3]),
                  1.0f - 2.0f * (quat[1] * quat[1] + quat[2] * quat[2]));
}

void get_angle(const fp32 quat[4], fp32 *yaw, fp32 *pitch, fp32 *roll)
{
    *yaw   = get_yaw(quat);
    *pitch = get_pitch(quat);
    *roll  = get_roll(quat);
}

fp32 get_carrier_gravity(void)
{
    return 9.81f;
}
