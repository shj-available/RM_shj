//
// Created by sunhongji on 2024/10/24.
//
#include <usart.h>

class RC_Ctl_t
{
private:
    uint8_t buffer[18];
    uint8_t data[18];
public:
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

    float Linearmap(int16_t input_Min,int16_t input_Max,float output_Min,float output_Max,int16_t input ) {
        float k = (output_Max-output_Min)/(input_Max-input_Min);
        return (k*(input-input_Min)+output_Min);
    }
    void RemoteDataProcess(uint8_t *pData)
    {
        if(pData == NULL)
        {
            return;
        }
        int16_t ch0_ori = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;
        rc.ch0 = Linearmap(364,1684,0.0f,1.0f,ch0_ori);
        int16_t ch1_ori = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5))& 0x07FF;
        rc.ch1 = Linearmap(364,1684,0.0f,1.0f,ch1_ori);
        int16_t ch2_ori = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) | ((int16_t)pData[4] << 10)) & 0x07FF;
        rc.ch2 = Linearmap(364,1684,0.0f,1.0f,ch2_ori);
        int16_t ch3_ori = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;
        rc.ch3 = Linearmap(364,1684,0.0f,1.0f,ch3_ori);
        uint8_t s1_ori = ((pData[5] >> 4) & 0x000C) >> 2;
        uint8_t s2_ori = ((pData[5] >> 4) & 0x0003) >> 2;
        rc.s1 = (switch_state)(s1_ori+1);
        rc.s2 = (switch_state)(s2_ori+1);
        int16_t x_ori = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
        int16_t y_ori = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
        int16_t z_ori = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);
        mouse.x = Linearmap(0,32767,0.0f,1.0f,x_ori);
        mouse.y = Linearmap(0,32767,0.0f,1.0f,y_ori);
        mouse.z = Linearmap(0,32767,0.0f,1.0f,z_ori);
        uint8_t press_l_ori = pData[12];
        uint8_t press_r_ori = pData[13];
        uint16_t key_ori = (((int16_t)pData[15] << 8) | ((int16_t)pData[14]));
        mouse.press_l = (press_state)press_l_ori;
        mouse.press_r = (press_state)press_r_ori;
        key.W = (press_state)(key_ori & ((uint16_t)0x01<<0)>>0);
        key.S = (press_state)(key_ori & ((uint16_t)0x01<<1)>>1);
        key.A = (press_state)(key_ori & ((uint16_t)0x01<<2)>>2);
        key.D = (press_state)(key_ori & ((uint16_t)0x01<<3)>>3);
        key.Q = (press_state)(key_ori & ((uint16_t)0x01<<4)>>4);
        key.E = (press_state)(key_ori & ((uint16_t)0x01<<5)>>5);
        key.Shift = (press_state)(key_ori & ((uint16_t)0x01<<6)>>6);
        key.Ctrl = (press_state)(key_ori & ((uint16_t)0x01<<7)>>7);
    }

    void readDBUSdata() {
        HAL_UART_Receive_DMA(&huart1, buffer, 18);
        for (int i = 0; i < sizeof(buffer); i++){
            data[i] = buffer[i];
        }
        RemoteDataProcess(data);


    }
};



