//
// Created by sunhongji on 2024/10/25.
//
#include "main.h"
#include "can.h"
#include <string.h>

CAN_RxHeaderTypeDef rxHeader={514,0,0,0,8};
CAN_TxHeaderTypeDef txHeader = {512,0,0,0,8};
uint8_t rxData[8];
uint8_t txData[8];
uint32_t TxMAilBox;
int Speed;
int Speed_tar = 180;
int current = 112;

int GetSpeed(int speedH,int speedL) {
    return ((speedH << 8) + speedL);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if(hcan->Instance == hcan1.Instance) {
        HAL_StatusTypeDef statu_rx;
        HAL_StatusTypeDef statu_tx;
        HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rxHeader,rxData);
        Speed = GetSpeed(rxData[2],rxData[3]);
        memset(txData,0,sizeof(txData));
        txData[3] = current + (Speed - Speed_tar)/50;
        HAL_CAN_AddTxMessage(hcan,&txHeader,txData,&TxMAilBox);
    }
}