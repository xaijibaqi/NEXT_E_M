//
// Created by m on 23-10-30.
//

#ifndef NEXT_E_M_SERIAL_H
#define NEXT_E_M_SERIAL_H

#include "string"
#include <libserialport.h>
#include <cstring>
#include <stdio.h>
#include <stdlib.h>
#include "opencv2/opencv.hpp"

class SERIAL {

public:

//  串口通讯接受数据
    struct Robot{
        std::string Enemy_color="Red";
        float Yaw_angle=0;
        float Pitch_angle=0;
    };

//  串口通讯发送数据
    struct Send_data{
        float Yaw_angle=0;
        float Pitch_angle=0;
        std::string Fire_control="N";
    };

    Send_data serial_data;

    bool sp_ret;

    std::string msg;

    char buffer[25];

    struct sp_port *serPort;

    Robot robot1;

    //  数据处理
    float string_float(std::string a,int pos,int n);

    Robot read_serial_data();

    //开启设备
    bool open();

    //发送数据
    [[noreturn]] void send();

    //接受数据
    [[noreturn]] bool receive();

    SERIAL();

    ~SERIAL();

};


#endif //NEXT_E_M_SERIAL_H
