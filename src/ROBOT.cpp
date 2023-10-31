//
// Created by m on 23-10-31.
//

#include "../include/ROBOT.h"

float ROBOT::parabola(float x,float v){
    x=x*(acos(-1)/180);
    float k=0.01,g=9.8;
    float vx=v* cos(x),vy=v* sin(x);
    float t= log((1+(k*vy*vy)/g))/(k*g)+ atan(vy* sqrt(k/g))/ sqrt(k*g);
    float d= log(k*vx*t+1)/k;
    return d*1000;
}

void ROBOT::Trajectory_solution(){
//    printf("armor=%d ",armour_list.size());
    for (int i = 0; i < Armour_list.size(); ++i) {
        ARMOUR::Armour armor1=Armour_list[i];
        if (armor1.real_distance>24000){
            break;
        }
        int io=0;
        float distance=12118.48141,a1=0,a2=25,a3=50,a4=0,dif= abs(armor1.real_distance-distance);
        while (dif>1){
            io++;
            if (distance>armor1.real_distance){
                a3=a2;
                a2=(a1+a3)/2;
                distance= parabola(a2,16);
                dif= abs(armor1.real_distance-distance);
            } else{
                a1=a2;
                a2=(a1+a3)/2;
                distance= parabola(a2,16);
                dif= abs(armor1.real_distance-distance);
            }
        }
        std::cout<<"armour_type "<<armor1.lable_id<<" angle "<<a2<<" cycle_index "<<io<<"\n";
    }
}
