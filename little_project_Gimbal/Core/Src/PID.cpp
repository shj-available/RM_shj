//
// Created by sunhongji on 2024/10/26.
//

#include "PID.h"


PID::PID(float kb, float ki, float kd, float i_max, float out_max, float i_switch) {
    kp_ = kb, ki_ = ki, kd_ = kd;
    i_switch_ = i_switch;
    i_max_ = i_max,out_max_ = out_max;
}

float PID::calc(float ref, float fdb) {
    ref_ = ref, fdb_ = fdb;
    err_ = ref - fdb;
    if(err_ < i_switch_ && err_ > -i_switch_) {
        err_sum_ += err_;
    }
    err_dev_ = err_ - last_err_;
    last_err_ = err_;
    pout_ = kp_ * err_;
    iout_ = ki_ * err_sum_;
    if (iout_ > i_max_) {iout_ = i_max_;}
    else if (iout_ < -i_max_) {iout_ = -i_max_;}
    dout_ = kd_ * err_dev_;

    output_ = pout_ + iout_ + dout_;
    if (output_ > out_max_) {output_ = out_max_;}
    else if (output_ < -out_max_) {output_ = -out_max_;}

    return output_;
}

