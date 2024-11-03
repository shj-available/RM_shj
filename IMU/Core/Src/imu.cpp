//
// Created by sunhongji on 2024/11/2.
//


#include "imu.h"
#include "spi.h"
#include "stm32f4xx_hal.h"
#include "main.h"

uint8_t rx_data1[7];
uint8_t rx_data2[6];
int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;
float ACC[3];
float GYRO[3];
IMU::IMU() {
    acc_range = 1;
    gyro_range = 0;
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
void IMU::BMI088_WriteReg(uint8_t reg, uint8_t write_data) {
    uint8_t tx_data[2];
    tx_data[0] = reg;
    tx_data[1] = write_data;
    BMI088_ACCEL_NS_L();
    HAL_Delay(10);
    HAL_SPI_Transmit(&hspi1,tx_data, 2, 1000);
    BMI088_ACCEL_NS_H();
    acc_range = write_data;
}

void IMU::read_accel() {
    uint8_t tx_data = 0x80|0x12;
    float k;
    BMI088_ACCEL_NS_L();
    HAL_SPI_Transmit(&hspi1,&tx_data, 1, 1000);
    HAL_SPI_Receive(&hspi1, rx_data1, 7, 1000);
    BMI088_ACCEL_NS_H();
    accX = (rx_data1[2] << 8) | rx_data1[1];
    accY = (rx_data1[4] << 8) | rx_data1[3];
    accZ = (rx_data1[6] << 8) | rx_data1[5];
    if(acc_range == 0) k = 9.8/32767.0*3.0;
    else if(acc_range == 1) k = 9.8/32767.0*6.0;
    else if(acc_range == 2) k = 9.8/32767.0*12.0;
    else if(acc_range == 3) k = 9.8/32767.0*24.0;
    ACC[0] = accX*k;
    ACC[1] = accY*k;
    ACC[2] = accZ*k;
    Accel_x = accX*k;
    Accel_y = accY*k;
    Accel_z = accZ*k;
}
void IMU::read_gyro() {
    uint8_t tx_data = 0x80|0x02;
    float k;
    BMI088_GYRO_NS_L();
    HAL_SPI_Transmit(&hspi1,&tx_data, 1, 1000);
    HAL_SPI_Receive(&hspi1, rx_data2, 6, 1000);
    BMI088_GYRO_NS_H();
    gyroX = (rx_data2[1] << 8) | rx_data2[0];
    gyroY = (rx_data2[3] << 8) | rx_data2[2];
    gyroZ = (rx_data2[5] << 8) | rx_data2[4];
    if(gyro_range == 0) k = 0.061;
    else if(gyro_range == 1) k = 0.0305;
    else if(gyro_range == 2) k = 0.0153;
    else if(gyro_range == 3) k = 0.0076;
    else if(gyro_range == 4) k = 0.0038;
    GYRO[0] = gyroX*k;
    GYRO[1] = gyroY*k;
    GYRO[2] = gyroZ*k;
    Gyro_x = gyroX*k;
    Gyro_y = gyroY*k;
    Gyro_z = gyroZ*k;
}


