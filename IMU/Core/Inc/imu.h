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
    float Accel_x;
    float Accel_y;
    float Accel_z;
    float Gyro_x;
    float Gyro_y;
    float Gyro_z;
    uint8_t acc_range;
    uint8_t gyro_range;

    void BMI088_WriteReg(uint8_t reg, uint8_t write_data);
    void Read_reg(uint8_t reg, uint8_t *read_data);
    void read_accel();
    void read_gyro();

private:
    void BMI088_ACCEL_NS_L(void);
    void BMI088_ACCEL_NS_H(void);
    void BMI088_GYRO_NS_L(void);
    void BMI088_GYRO_NS_H(void);



};



#endif //IMU_H
