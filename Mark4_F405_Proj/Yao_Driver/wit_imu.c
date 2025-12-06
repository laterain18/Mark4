#include "wit_imu.h"

// 全局变量定义
IMUData imuData;

// 计算校验和函数
unsigned char calculateChecksum(unsigned char data[])
{
    unsigned char checksum = 0;
    for (int i = 0; i < 10; i++)
    {
        checksum += data[i];
    }
    return checksum;
}

// 解析加速度数据函数
int parseAccelData(unsigned char data[], AccelData *accelData)
{
    if (calculateChecksum(data) != data[10])
    {
        return 0;
    }
    short accelX_raw = (data[3] << 8) | data[2];
    accelData->accelX = (float)accelX_raw / 32768.0 * 16.0;
    short accelY_raw = (data[5] << 8) | data[4];
    accelData->accelY = (float)accelY_raw / 32768.0 * 16.0;
    short accelZ_raw = (data[7] << 8) | data[6];
    accelData->accelZ = (float)accelZ_raw / 32768.0 * 16.0;
    short temp_raw = (data[9] << 8) | data[8];
    accelData->temperature = (float)temp_raw / 100.0;
    return 1;
}

// 解析角速度数据函数
int parseGyroData(unsigned char data[], GyroData *gyroData)
{
    if (calculateChecksum(data) != data[10])
    {
        return 0;
    }
    short gyroX_raw = (data[3] << 8) | data[2];
    gyroData->gyroY = (float)gyroX_raw / 32768.0 * 2000.0;
    short gyroY_raw = (data[5] << 8) | data[4];
    gyroData->gyroX = (float)gyroY_raw / 32768.0 * 2000.0;
    short gyroZ_raw = (data[7] << 8) | data[6];
    gyroData->gyroZ = (float)gyroZ_raw / 32768.0 * 2000.0;
    gyroData->voltage = 0.0;
    return 1;
}

// 解析角度数据函数
int parseAngleData(unsigned char data[], AngleData *angleData)
{
    if (calculateChecksum(data) != data[10])
    {
        return 0;
    }
    short roll_raw = (data[3] << 8) | data[2];
    angleData->pitch = (float)roll_raw / 32768.0 * 180.0 - offset_pitch;
    short pitch_raw = (data[5] << 8) | data[4];
    angleData->roll = (float)pitch_raw / 32768.0 * 180.0 - offset_roll;
    short yaw_raw = (data[7] << 8) | data[6];
    angleData->yaw = (float)yaw_raw / 32768.0 * 180.0;
    angleData->version = (data[9] << 8) | data[8];
    return 1;
}

// 解析磁场数据函数
int parseMagData(unsigned char data[], MagData *magData)
{
    if (calculateChecksum(data) != data[10])
    {
        return 0;
    }
    short magX_raw = (data[3] << 8) | data[2];
    magData->magX = (float)magX_raw;
    short magY_raw = (data[5] << 8) | data[4];
    magData->magY = (float)magY_raw;
    short magZ_raw = (data[7] << 8) | data[6];
    magData->magZ = (float)magZ_raw;
    short temp_raw = (data[9] << 8) | data[8];
    magData->temperature = (float)temp_raw / 100.0;
    return 1;
}

/**
 * @brief 交换IMUData结构体中X、Y轴的角速度和roll、pitch角度值
 * @param imu_data_ptr 指向IMUData结构体的指针
 * @retval None
 */
void swap_imu_xy(IMUData *imu_data_ptr)
{
    if (imu_data_ptr == NULL) {
        return; // 安全检查，避免空指针
    }
    
    // 交换角速度X和Y的值
    float temp_gyro = imu_data_ptr->gyro.gyroX;
    imu_data_ptr->gyro.gyroX = imu_data_ptr->gyro.gyroY;
    imu_data_ptr->gyro.gyroY = temp_gyro;
    
    // 交换角度roll和pitch的值
    float temp_angle = imu_data_ptr->angle.roll;
    imu_data_ptr->angle.roll = imu_data_ptr->angle.pitch - offset_roll;
    imu_data_ptr->angle.pitch = temp_angle - offset_pitch;
}

// 解析IMU数据函数
// 可以选择性的开启需要解析的数据以增加程序的性能，这里我暂且只开启动角度
int parseIMUData(unsigned char data[], IMUData *imuData)
{
    // if (!parseAccelData(data, &imuData->accel)) return 0;
    if (!parseGyroData(data + 11, &imuData->gyro)) return 0;
    if (!parseAngleData(data + 22, &imuData->angle)) return 0;
    // if (!parseMagData(data + 33, &imuData->mag)) return 0;

    // 实际使用xy轴需要互换
    // swap_imu_xy(&imuData);

    return 1;
}
