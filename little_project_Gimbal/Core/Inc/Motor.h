//
// Created by sunhongji on 2024/11/10.
//

#ifndef MOTOR_H
#define MOTOR_H
#include "PID.h"
#include<can.h>
#include <cstdint>

enum Model{M2006,M3508,M6020,M3508_butt};

class Motor {
private:

public:
    Model model;
    uint32_t id;
    float reduction_ratio = 1;
    CAN_RxHeaderTypeDef rxHeader= {0x204+id,0,0,0,8};

    uint8_t rx_data[8];
    uint16_t tx_data;
    int output;

    float angle = -30;
    float ori_angle;
    float ori_angle_last;
    int16_t ori_speed;
    float speed;
    float current;
    int flag = 0;

    PID speed_controller = PID(180,0,0,1000,25000,1);
    PID angle_controller = PID(8,0.01,0,100,200,3);

    Motor(Model model_,uint32_t ID);
    void read_data();
    void decode_data();
    void operate();
    uint16_t control_speed(float speed_ref);
    uint16_t control_angle(float angle_ref);
    uint16_t control_angle(float angle_ref,float angle_fdb);
    uint16_t feedforward(float tar_angle);
    void tuning_PID(PID controller,float Kp,float Ki,float Kd,float I_max,float Out_max);
};



#endif //MOTOR_H
