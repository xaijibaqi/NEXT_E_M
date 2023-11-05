//
// Created by m on 23-10-30.
//

#ifndef NEXT_E_M_ARMOUR_H
#define NEXT_E_M_ARMOUR_H

#include "iostream"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <vector>
#include <algorithm>

class ARMOUR {
public:
    ARMOUR();
    ~ARMOUR();

    std::vector<std::string> class_names_={"Base","1","2","3","4","5","Guard","Outpost","Negative"};


    struct Light_bar{
        cv::Point2f top;
        cv::Point2f centre;
        cv::Point2f bottom;
        float angle;
        float lenth;

    };

    struct Armour{
        Light_bar light_right;
        Light_bar light_left;
        cv::Point2f center;
        float Distance_centers;
        std::string armour_type="SMALL";
        std::string lable_id;
        float confidence;
        cv::Mat Rvec;
        cv::Mat_<float> Tvec;


        float real_distance;
        float shoot_pitch;
        float shoot_yaw;
    };

    std::vector<Armour> Armour_list;

    cv::Mat Cap_img;
    cv::Mat Color_processing_img;
    cv::Mat light_img;



    std::string Enemy_color="Blue";

    void Image_processing();

    void find_light();

    void extractNumbers();

    void distance_measurement();



    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 1869.07419731815, 0, 664.699824168277,
            0, 1867.898354, 518.0525069,
            0, 0, 1);

    cv::Mat dist_coeffs = (cv::Mat_<double>(5, 1) << -0.163116073466183, 0.255155351,
            0, 0, 0);

};

#endif //NEXT_E_M_ARMOUR_H
