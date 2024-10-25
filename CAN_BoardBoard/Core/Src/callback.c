//
// Created by sunhongji on 2024/10/25.
//
#include "main.h"
#include "can.h"
#include <GPIO.h>

CAN_RxHeaderTypeDef rxHeader={512,0,0,0,1};
CAN_TxHeaderTypeDef txHeader = {512,0,0,0,1};
uint8_t rxData;
;
uint32_t TxMAilBox;
//接收板
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if(hcan->Instance == hcan1.Instance) {
        HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rxHeader,&rxData);
        if(rxData == 0x01) {
            HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
        }
        rxData = 0x00;
    }
}
//发送板
void EXTI2_IRQHandler(void)
{
    /* USER CODE BEGIN EXTI2_IRQn 0 */

    /* USER CODE END EXTI2_IRQn 0 */
    HAL_GPIO_EXTI_IRQHandler(KEY_Pin);
    /* USER CODE BEGIN EXTI2_IRQn 1 */
    uint8_t txData = 0x01;
    HAL_CAN_AddTxMessage(&hcan1,&txHeader,&txData,&TxMAilBox);
    /* USER CODE END EXTI2_IRQn 1 */
}