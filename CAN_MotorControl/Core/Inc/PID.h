//
// Created by sunhongji on 2024/10/26.
//

#ifndef PID_H
#define PID_H



class PID{
public:
    PID(float kb, float ki, float kd, float i_max, float out_max);
    float calc(float ref, float fdb);
private:
    float kp_,ki_, kd_;
    float i_max_,out_max_;
    float output_;
    float ref_, fdb_;
    float err_,err_sum_, last_err_, err_dev_;
    float pout_,iout_, dout_;
};



#endif //PID_H
