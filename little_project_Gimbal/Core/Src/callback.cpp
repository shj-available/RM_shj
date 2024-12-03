//
// Created by sunhongji on 2024/11/10.
//

#include <main.h>
#include <usart.h>
#include <can.h>
#include <spi.h>
#include <tim.h>
#include <iwdg.h>
#include <RC.h>
#include "imu.h"
#include "Motor.h"
#include "cstring"

extern RC rc;
extern IMU imu;
extern Motor pitch;
extern Motor yaw;
//遥控器数据
uint8_t rxData_rc[18];
//imu数据
uint8_t rxData_accel[7];
uint8_t rxData_gyro[6];
//电机数据
uint8_t rxData_pitch[8];
uint8_t rxData_yaw[8];
uint16_t txData_pitch,txData_yaw;
uint8_t txData_motor[8];
uint32_t TxMAilBox;
CAN_TxHeaderTypeDef txHeader = {0x1FF,0,0,0,8};

//状态量
float pitch_angle;
float yaw_angle;
float pitch_speed;
float yaw_speed;
float accel[3];
float gyro[3];
//目标值
float tar_pitch_angle = -30;
float tar_yaw_speed = 0;
//PID参数
float p_a_kp = 12,p_a_ki = 0,p_a_kd = 0,p_a_im = 100,p_a_om = 200;
float p_s_kp = 100,p_s_ki = 0,p_s_kd = 10,p_s_im = 1000,p_s_om = 20000;
float y_a_kp = 0,y_a_ki = 0,y_a_kd = 0,y_a_im = 100,y_a_om = 200;
float y_s_kp = 0,y_s_ki = 0,y_s_kd = 0,y_s_im = 100,y_s_om = 200;

float Linearmap(float input_Min, float input_Max, float output_Min, float output_Max, float input) {
    float k = (output_Max-output_Min)/(input_Max-input_Min);
    return (k*(input-input_Min)+output_Min);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == htim6.Instance){
        //喂狗
        HAL_IWDG_Refresh(&hiwdg);

        //DBUs数据解码
        /*
        rc.decode_DBUSdata(rxData_rc);
        tar_pitch_angle = Linearmap(0,1,-30,30,rc.rc.ch2);
        tar_yaw_speed = Linearmap(0,1,-30,30,rc.rc.ch3);
        */
        //imu数据解码
        imu.read_accel();
        imu.read_gyro();
        imu.decode_accel();
        imu.decode_gyro();
        for(int i = 0;i<3;i++) {
            accel[i] = imu.Accel[i];
            gyro[i] = imu.Gyro[i];
        }
        imu.pose_solution();

        //实时调整PID参数，仅快速调试时使用
        pitch.tuning_PID(pitch.angle_controller,p_a_kp,p_a_ki,p_a_kd,p_a_im,p_a_om);
        pitch.tuning_PID(pitch.speed_controller,p_s_kp,p_s_ki,p_s_kd,p_s_im,p_s_om);
        yaw.tuning_PID(yaw.angle_controller,y_a_kp,y_a_ki,y_a_kd,y_a_im,y_a_om);
        yaw.tuning_PID(yaw.speed_controller,y_s_kp,y_s_ki,y_s_kd,y_s_im,y_s_om);
        //电机数据解码
        if(pitch.flag != 3) return;
        pitch_angle = pitch.angle;
        pitch_speed = pitch.speed;
        yaw.decode_data();
        yaw_angle = yaw.angle;
        yaw_speed = yaw.speed;
        //电机输出
        txData_pitch = pitch.control_angle(tar_pitch_angle,imu.pitch);
        txData_pitch += pitch.feedforward(tar_pitch_angle);
        txData_yaw = yaw.control_speed(tar_yaw_speed);
        memset(txData_motor,0,sizeof(txData_motor));
        txData_motor[2*pitch.id-2] = txData_pitch >> 8;
        txData_motor[2*pitch.id-1] = txData_pitch;
        txData_motor[2*yaw.id-2] = txData_yaw >> 8;
        txData_motor[2*yaw.id-1] = txData_yaw;
        HAL_CAN_AddTxMessage(&hcan1,&txHeader,txData_motor,&TxMAilBox);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if (huart == &huart3){
        rc.read_DBUSdata();
        for (int i = 0; i < sizeof(rc.data); i++){
            rxData_accel[i] = imu.rx_data_accel[i];
        }
    }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == hspi1.Instance) {
        imu.read_accel();
        for (int i = 0; i < sizeof(imu.rx_data_accel); i++) {
            rxData_accel[i] = imu.rx_data_accel[i];
        }
        imu.read_gyro();
        for (int i = 0; i < sizeof(imu.rx_data_gyro); i++) {
            rxData_gyro[i] = imu.rx_data_gyro[i];
        }
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if(hcan->Instance == hcan1.Instance) {
        if (pitch.flag != 3) {
            pitch.flag++;
            pitch.angle = -30;
        }

        HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&pitch.rxHeader,pitch.rx_data);
        for(int i = 0;i<8;i++) {
            rxData_pitch[i] = pitch.rx_data[i];
        }
        pitch.decode_data();

        HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&yaw.rxHeader,yaw.rx_data);
        for(int i = 0;i<8;i++) {
            rxData_yaw[i] = yaw.rx_data[i];
        }
        yaw.decode_data();
    }
}
