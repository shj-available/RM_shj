//
// Created by sunhongji on 2024/11/2.
//


#include "imu.h"
#include "spi.h"
#include "stm32f4xx_hal.h"
#include "main.h"
#include "cmath"

IMU::IMU() {
    acc_range = 2;
    gyro_range = 0;
}
void IMU::BMI088_WriteReg(uint8_t reg, uint8_t write_data) {
    uint8_t tx_data[2];
    tx_data[0] = reg;
    tx_data[1] = write_data;
    HAL_Delay(2);
    HAL_SPI_Transmit(&hspi1,tx_data, 2, 1000);
}
void IMU::IMU_init() {
    // Soft Reset ACCEL
    BMI088_ACCEL_NS_L();
    BMI088_WriteReg(0x7E, 0xB6); // Write 0xB6 to ACC_SOFTRESET(0x7E)
    HAL_Delay(1);
    BMI088_ACCEL_NS_H();

    // Soft Reset GYRO
    BMI088_GYRO_NS_L();
    BMI088_WriteReg(0x14, 0xB6); // Write 0xB6 to GYRO_SOFTRESET(0x14)
    HAL_Delay(30);
    BMI088_GYRO_NS_H();

    // Switch ACCEL to Normal Mode
    BMI088_ACCEL_NS_L();
    HAL_Delay(1);
    BMI088_WriteReg(0x7D, 0x04); // Write 0x04 to ACC_PWR_CTRL(0x7D)
    HAL_Delay(1);
    BMI088_ACCEL_NS_H();

    // Set ACCEL RANGE
    BMI088_ACCEL_NS_L();
    HAL_Delay(1);
    BMI088_WriteReg(0x41, 0x02); // Write 0x04 to ACC_PWR_CTRL(0x7D)
    HAL_Delay(1);
    BMI088_ACCEL_NS_H();
}

void IMU::BMI088_ACCEL_NS_L(void){
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}
void IMU::BMI088_ACCEL_NS_H(void){
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}
void IMU::BMI088_GYRO_NS_L(void){
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
}
void IMU::BMI088_GYRO_NS_H(void){
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
}

void IMU::read_accel() {
    uint8_t tx_data = 0x80|0x12;
    BMI088_ACCEL_NS_L();
    HAL_SPI_Transmit(&hspi1,&tx_data, 1, 1000);
    HAL_SPI_Receive(&hspi1, rx_data_accel, 7, 1000);
    BMI088_ACCEL_NS_H();

}
void IMU::read_gyro() {
    uint8_t tx_data = 0x80|0x02;
    BMI088_GYRO_NS_L();
    HAL_SPI_Transmit(&hspi1,&tx_data, 1, 1000);
    HAL_SPI_Receive(&hspi1, rx_data_gyro, 6, 1000);
    BMI088_GYRO_NS_H();

}
void IMU::decode_accel() {
    int16_t accX, accY, accZ;
    float k;
    accX = (rx_data_accel[2] << 8) | rx_data_accel[1];
    accY = (rx_data_accel[4] << 8) | rx_data_accel[3];
    accZ = (rx_data_accel[6] << 8) | rx_data_accel[5];
    if(acc_range == 0) k = 9.8/32767.0*3.0;
    else if(acc_range == 1) k = 9.8/32767.0*6.0;
    else if(acc_range == 2) k = 9.8/32767.0*12.0;
    else if(acc_range == 3) k = 9.8/32767.0*24.0;
    Accel[0] = accX*k;
    Accel[1] = accY*k;
    Accel[2] = accZ*k;
    Accel_x = accX*k;
    Accel_y = accY*k;
    Accel_z = accZ*k;
}
void IMU::decode_gyro() {
    int16_t gyroX, gyroY, gyroZ;
    float k;
    gyroX = (rx_data_gyro[1] << 8) | rx_data_gyro[0];
    gyroY = (rx_data_gyro[3] << 8) | rx_data_gyro[2];
    gyroZ = (rx_data_gyro[5] << 8) | rx_data_gyro[4];
    if(gyro_range == 0) k = 0.061;
    else if(gyro_range == 1) k = 0.0305;
    else if(gyro_range == 2) k = 0.0153;
    else if(gyro_range == 3) k = 0.0076;
    else if(gyro_range == 4) k = 0.0038;
    Gyro[0] = gyroX*k;
    Gyro[1] = gyroY*k;
    Gyro[2] = gyroZ*k;
    Gyro_x = gyroX*k;
    Gyro_y = gyroY*k;
    Gyro_z = gyroZ*k;
}

void IMU::pose_solution() {
    float roll_a, pitch_a;
    float roll_g, pitch_g, yaw_g;
    float gyro_roll, gyro_pitch, gyro_yaw;
    float deg_rad = 3.1415926/180.0;
    float delta_t = 0.001;//second
    float r = roll*deg_rad, p = pitch*deg_rad, y = yaw*deg_rad;
    roll_a = atan2f(Accel_y,Accel_z)/deg_rad;
    pitch_a = -atan2f(Accel_x,sqrt(Accel_y*Accel_y+Accel_z*Accel_z))/deg_rad;
    pitch_acc = pitch_a;
    gyro_roll = Gyro_x + sin(p)*sin(r)/cos(p)*Gyro_y + sin(p)*cos(r)/cos(p)*Gyro_z;
    gyro_pitch = cos(r)*Gyro_y - sin(r)*Gyro_z;
    gyro_yaw = sin(r)/cos(p)*Gyro_y + cos(r)/cos(p)*Gyro_z;
    roll_g = roll + gyro_roll*delta_t;
    pitch_g = pitch + gyro_pitch*delta_t;
    pitch_gyro_pure += gyro_pitch*delta_t;

    pitch_gyro = pitch_g;
    yaw_g = yaw + gyro_yaw*delta_t;
    roll = K*roll_a + (1-K)*roll_g;
    pitch = K*pitch_a + (1-K)*pitch_g;
    yaw = yaw_g;

}


