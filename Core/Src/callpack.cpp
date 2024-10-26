//
// Created by sunhongji on 2024/10/25.
//
#include "main.h"
#include "can.h"
#include "string.h"
#include "PID.h"

extern PID speed_pid_controller;
extern PID angle_pid_controller;

CAN_RxHeaderTypeDef rxHeader={0x202,0,0,0,8};
CAN_TxHeaderTypeDef txHeader = {0x200,0,0,0,8};
uint8_t rxData[8];
uint8_t txData[8];
uint32_t TxMAilBox;
int16_t Speed;
float angle;
float angle_delta;
float angle_last;
float angle_rele;
int Speed_ref = 180;
float angle_ref = 160
	;
int current = 100;

int16_t GetSpeed(int8_t speedH,int8_t speedL) {
    return ((speedH << 8) + speedL);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if(hcan->Instance == hcan1.Instance) {
        HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rxHeader,rxData);
        Speed = GetSpeed(rxData[2],rxData[3]);
        angle_last = angle_rele;
        angle_rele = GetSpeed(rxData[0],rxData[1]) * 360.0 / 8192.0;
        if (angle_rele - angle_last > 180){angle_delta = angle_rele - angle_last - 360.0;}
        else if(angle_rele - angle_last < -180){angle_delta = angle_rele - angle_last + 360.0;}
        else{angle_delta = angle_rele - angle_last;}
        angle += angle_delta;
        //速度单环控制代码
        /*current = speed_pid_controller.calc(Speed_ref,Speed);*/
        //角度速度双环控制代码

        //Speed_ref = angle_pid_controller.calc(angle_ref,angle);
        current = angle_pid_controller.calc(angle_ref,angle);

        memset(txData,0,sizeof(txData));
        txData[3] = current;
        txData[2] = current >> 8;
        HAL_CAN_AddTxMessage(hcan,&txHeader,txData,&TxMAilBox);
    }
}