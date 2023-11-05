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

bool ROBOT::Target_selection(){
    int target=0;
    for (int i = 0; i < Armour_list.size(); ++i) {
        ARMOUR::Armour armour1= Armour_list[i];
         if (std::find(Target_names_.begin(),Target_names_.end(),armour1.armour_type)<std::find(Target_names_.begin(),Target_names_.end(),Armour_list[target].armour_type)){
             target=i;
         } else{
             break;
         }
    }
    target_armour=Armour_list[target];
    qwer = true;
};

void ROBOT::Coordinate_transformation() {

    std::cout<<"\n Tvec"<<target_armour.Tvec;
//    std::cout<<"\n Tvec_now"<<target_Tvec;
    target_Rvec=target_armour.Rvec;
    target_Tvec=target_armour.Tvec;

    //    求变化矩阵


    cv::Mat_<float> Realistic_coordinate(3,3);
    Realistic_coordinate=cv::Mat::zeros(3,3,CV_8UC3);
    Realistic_coordinate.at<float>(0,0)= 1;
    Realistic_coordinate.at<float>(1,1)= cos(-Pitch_angle*3.1415/180);
    Realistic_coordinate.at<float>(1,2)= sin(-Pitch_angle*3.1415/180);
    Realistic_coordinate.at<float>(2,1)= -sin(-Pitch_angle*3.1415/180);
    Realistic_coordinate.at<float>(2,2)= cos(-Pitch_angle*3.1415/180);

    cv::Mat_<float> rotMat(3, 3);
    Rodrigues(target_Rvec, rotMat);
    // 旋转向量转成旋转矩阵

    P_oc = Realistic_coordinate * target_Tvec;
    std::cout<<"\n p_oc"<<P_oc;

    // 求解相机的世界坐标，得出p_oc的第三个元素即相机到物体的距离即深度信息，单位是mm

}

void ROBOT::Trajectory_solution(){

    float target_x,target_y,target_z;
    target_x=P_oc.at<float>(0,0);
    target_y=P_oc.at<float>(0,1);
    target_z=P_oc.at<float>(0,2);






    serial_data1.Yaw_angle= atan(-target_x/target_z)*180/3.141592;

//        std::ifstream infile;
//        infile.open("file1.dat");
//        infile >> ;
    if (serial_data1.Yaw_angle>45){
        serial_data1.Yaw_angle=serial_data1.Yaw_angle-90;
//        std::cout<<"\n"<<target_Tvec<<"\n"<<"\n------------------------------------------------------\n";
    }
    if (serial_data1.Yaw_angle<-45){
        serial_data1.Yaw_angle=serial_data1.Yaw_angle+90;
//        std::cout<<"\n"<<target_Tvec<<"\n"<<"\n------------------------------------------------------\n";
    }


        std::cout<<"\nPitch_angle "<<serial_data1.Pitch_angle<<" Yaw_angle "<<serial_data1.Yaw_angle<<" type "<< target_armour.lable_id<<"\n";

//    }
}


