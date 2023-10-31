//
// Created by m on 23-10-31.
//

#ifndef NEXT_E_M_ROBOT_H
#define NEXT_E_M_ROBOT_H

#include "iostream"
#include "ARMOUR.h"

class ROBOT {
public:
    std::string Enemy_color="Red";
    float Yaw_angle=0;
    float Pitch_angle=0;
    float Spring_velocity=16;

    void Trajectory_solution();

    float parabola(float x,float v);

    std::vector<ARMOUR::Armour> Armour_list;

};


#endif //NEXT_E_M_ROBOT_H
