//
// Created by sunhongji on 2024/11/2.
//

#ifndef IMU_H
#define IMU_H
#include <stdint.h>
class IMU {
public:
    IMU();
    void IMU_init();
    float Accel_x,Accel_y,Accel_z;
    float Accel[3];
    float Gyro_x,Gyro_y,Gyro_z;
    float Gyro[3];
    float pitch,roll,yaw,K = 0.003;//结算出的三轴角度及加速度权重K
    float pitch_acc,pitch_gyro,pitch_gyro_pure = -30;
    uint8_t acc_range;
    uint8_t gyro_range;
    uint8_t rx_data_accel[7];
    uint8_t rx_data_gyro[6];
    void BMI088_WriteReg(uint8_t reg, uint8_t write_data);
    void BMI088_ACCEL_NS_L(void);
    void BMI088_ACCEL_NS_H(void);
    void BMI088_GYRO_NS_L(void);
    void BMI088_GYRO_NS_H(void);
    void read_accel();
    void read_gyro();
    void decode_accel();
    void decode_gyro();
    void pose_solution();

private:



};



#endif //IMU_H
