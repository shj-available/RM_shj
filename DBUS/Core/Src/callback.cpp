//
// Created by sunhongji on 2024/10/24.
//

#include <main.h>
#include <usart.h>
#include <RC.h>

extern RC_Ctl_t rc;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if (huart == &huart1){
        rc.readDBUSdata();
    }
}