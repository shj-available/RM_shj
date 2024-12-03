//
// Created by sunhongji on 2024/11/10.
//

#include "../Inc/Motor.h"
#include "can.h"
Motor::Motor(Model model_,uint32_t ID):model(model_),id(ID) {
    if (model_ == M2006) reduction_ratio = 36;
    else if (model_ == M3508) reduction_ratio = 3591.0/187;
}
void Motor::read_data() {
    HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&rxHeader,rx_data);

};
void Motor::decode_data() {
    //计算电机输出轴角度
    float ori_angle_delta;
    ori_angle_last = ori_angle;
    ori_angle = ((rx_data[0] << 8) + rx_data[1])* 360.0 / 8192.0;
    if (ori_angle - ori_angle_last > 180) ori_angle_delta = ori_angle - ori_angle_last - 360.0;
    else if (ori_angle - ori_angle_last < -180) ori_angle_delta = ori_angle - ori_angle_last + 360.0;
    else ori_angle_delta = ori_angle - ori_angle_last;
    angle = angle + ori_angle_delta/reduction_ratio;
    //计算输出轴速度
    ori_speed = ((rx_data[2] << 8) + rx_data[3]);
    speed = ori_speed/reduction_ratio;
}
void Motor::operate() {
    read_data();
    decode_data();
}
uint16_t Motor::control_speed(float speed_ref) {
    tx_data = speed_controller.calc(speed_ref, speed);
    return tx_data;
}
uint16_t Motor::control_angle(float angle_ref) {

    float speed_ref = angle_controller.calc(angle_ref, angle);
    output = speed_controller.calc(speed_ref, speed);
    tx_data = output;
    return tx_data;
}
uint16_t Motor::control_angle(float angle_ref,float angle_fdb) {
    float speed_ref = angle_controller.calc(angle_ref, angle_fdb);
    output = speed_controller.calc(speed_ref, speed);
    tx_data = output;
    return tx_data;

}
void Motor::tuning_PID(PID controller, float Kp, float Ki, float Kd, float I_max, float Out_max) {
    controller.kp_ = Kp;
    controller.ki_ = Ki;
    controller.kd_ = Kd;
    controller.i_max_ = I_max;
    controller.out_max_ = Out_max;
}
uint16_t Motor::feedforward(float tar_angle) {
    uint16_t output;
    float k = 3.1415926/180;
    int16_t u_max = 1800;
    int16_t max_angle = -40;
    int16_t feedforward = u_max*(1-(tar_angle-max_angle)*k*(tar_angle-max_angle)*k/2);
    output = feedforward;
    return output;
}



Motor pitch=Motor(M6020,1);
Motor yaw=Motor(M6020,2);