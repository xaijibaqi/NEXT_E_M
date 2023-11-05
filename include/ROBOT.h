//
// Created by m on 23-10-31.
//

#ifndef NEXT_E_M_ROBOT_H
#define NEXT_E_M_ROBOT_H

#include "iostream"
#include "ARMOUR.h"
#include "SERIAL.h"
//#include <pcl/io/pcd_io.h>
//#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>

class ROBOT {
public:

    std::string Enemy_color="Red";
    SERIAL::Send_data serial_data1;
    float Yaw_angle=0;
    float Pitch_angle=0;
    float Spring_velocity=16;
    float g=9.8;
    float k=0.1;




    std::vector<std::string> Target_names_={"Base","Outpost","1","Guard","2","3","4","5"};
    int shoot_mod=0;

    ARMOUR::Armour target_armour;
    cv::Mat P_oc;
    bool qwer= false;
    cv::Mat target_Rvec;
    cv::Mat_<float > target_Tvec;

    void Trajectory_solution();

    float parabola(float x,float v);

    void Coordinate_transformation();

    bool Target_selection();


    std::vector<ARMOUR::Armour> Armour_list;

};


#endif //NEXT_E_M_ROBOT_H
