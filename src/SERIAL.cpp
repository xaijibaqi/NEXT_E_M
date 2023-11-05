//
// Created by m on 23-10-30.
//

#include "../include/SERIAL.h"

float SERIAL::string_float(std::string a, int pos, int n) {
    if (a.size()<pos+n){
        return 0;
    }
    std::string aa(a);
    std::string aaa(aa,pos,n);
    float angle=atof(aaa.c_str());
    return angle;
}

SERIAL::Robot SERIAL::read_serial_data() {

    Robot robot1;

    if(buffer[0]==66){
        robot1.Enemy_color="Blue";
    }

//    yaw_angle
    float Yaw_angle= string_float(buffer,3,7);

    if (buffer[2]==45){
        Yaw_angle*=-1;
    }
    Yaw_angle=Yaw_angle*180/3.141592;
    robot1.Yaw_angle=Yaw_angle;

//    pitch_angle
    float Pitch_angle= string_float(buffer,11,6);
    if (buffer[11]==45){
        Pitch_angle*=-1;
    }
    Pitch_angle=Pitch_angle*180/3.141592;
    robot1.Pitch_angle=Pitch_angle;

    return robot1;
}

SERIAL::SERIAL() {
    sp_ret = open();
}

bool SERIAL::open() {
    sp_return ret = sp_get_port_by_name("/dev/ttyACM0", &serPort);
    if(ret != SP_OK)sp_get_port_by_name("/dev/ttyACM1", &serPort);
    ret = sp_open(serPort,SP_MODE_READ_WRITE);
    if(ret != SP_OK) return false;
    sp_set_baudrate(serPort,115200);
    sp_set_bits(serPort, 8);
    sp_set_parity(serPort,SP_PARITY_NONE);
    sp_set_stopbits(serPort, 1);
    std::cout<<"open serial\n";
    return true;
}

[[noreturn]] void SERIAL::send(){
    while (true){
        msg = "A";msg += "Y";
        if(serial_data.Yaw_angle>0)msg += "-";
        else msg += "+";
        msg += cv::format("%06.2f", abs(serial_data.Yaw_angle=serial_data.Yaw_angle));
        msg += "P";
        if(serial_data.Pitch_angle>0)msg += "+";
        else msg += "-";
        msg += cv::format("%06.2f",abs(serial_data.Pitch_angle));
        if(abs(serial_data.Yaw_angle) < 5 && abs(serial_data.Pitch_angle) < 5) msg += "F";
        else msg += "N";
        msg += "E";
        if (serial_data.Yaw_angle!=0){
            std::cout<<"\n"<<msg<<"\n";
        }
        sp_blocking_write(serPort,msg.c_str(),19,0);
    }
}

[[noreturn]] bool SERIAL::receive() {
    while(true){
        char sign;
//        sign='A';//DEBUG
//        printf("\nbegan receive");
        sp_nonblocking_read(serPort,&sign,1);
        if(sign!='A'){
            sp_nonblocking_read(serPort,&buffer,24);
//            printf("\nread=%s",buffer);
            robot1=read_serial_data();
//            std::cout<<"\n"<<buffer<<"\n";
        } else{
            printf("no data");
        }
    }
}

SERIAL::~SERIAL() = default;
