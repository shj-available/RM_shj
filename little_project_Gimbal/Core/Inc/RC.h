//
// Created by sunhongji on 2024/11/10.
//

#ifndef RC_H
#define RC_H
#include <cstdint>

class RC {
private:

public:
    uint8_t buffer[18];
    uint8_t data[18];
    enum switch_state{UP,MID,DOWN};
    enum press_state{RELEASE,PRESS};
    struct
    {
        float ch0;
        float ch1;
        float ch2;
        float ch3;
        switch_state s1;
        switch_state s2;
    }rc;

    struct
    {
        float x;
        float y;
        float z;
        press_state press_l;
        press_state press_r;
    }mouse;

    struct
    {
        press_state W,A,D,S,Q,E,Shift,Ctrl;
    }key;

    float Linearmap(int16_t input_Min,int16_t input_Max,float output_Min,float output_Max,int16_t input );
    void read_DBUSdata();
    void decode_DBUSdata(uint8_t *pData);
};




#endif //RC_H
