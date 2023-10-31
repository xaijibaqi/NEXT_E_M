//
// Created by m on 23-10-30.
//

#include "../include/ARMOUR.h"

ARMOUR::ARMOUR() {

}
ARMOUR::~ARMOUR() {

}

void ARMOUR::Image_processing() {

    cv::Mat img =Cap_img.clone();
    std::vector<cv::Mat> channels;

    split(img,channels);

    cv::Mat blue,green,red;

    blue=channels[0];
    green=channels[1];
    red=channels[2];
//    imwrite("/home/m/CLionProjects/untitled/video_test/blue.jpg", blue);
//    imwrite("/home/m/CLionProjects/untitled/video_test/red.jpg", red);
//    imwrite("/home/m/CLionProjects/untitled/video_test/green.jpg", green);

    add(red/3,green/3,green);
    add(blue/3,green,green);
//    std::cout<<green<<std::endl;

    if (Enemy_color=="Blue"){

        img=blue;

    }
    else if (Enemy_color=="Red"){

        img=red;

    }

    threshold(img,img,150,255,cv::THRESH_BINARY);

    Color_processing_img=img.clone();

}

void ARMOUR::find_light() {

    light_img=Cap_img.clone();
    cv::Mat img=Color_processing_img;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<Light_bar> Light_list; //定义最小外接矩形集合
    cv::Point2f rect[4];
    std::vector<cv::Rect> boundRect(contours.size());  //定义外接矩形集合


    cv::findContours(img,contours,hierarchy,cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE);

//    cv::drawContours(light_img,contours,-1,cv::Scalar (0,255,0),-1,1);

    for(int i=0; i<contours.size(); i++) {

        cv::RotatedRect box1;
        box1.points(rect);
        box1 = minAreaRect(cv::Mat(contours[i]        ));  //计算每个轮廓最小外接矩形
        box1.points(rect);

        Light_bar light1;
        light1.centre=box1.center;
        light1.angle=box1.angle;
        cv::Point2f point1,point2;
        float area1 = contourArea(contours[i]);
//
        if (box1.size.width>box1.size.height){
            point1=(rect[0]/2+rect[1]/2);
            point2=(rect[0]/2+rect[3/2]);
            light1.lenth=box1.size.width;
        } else{
            point1=(rect[0]/2+rect[3]/2);
            point2=(rect[0]/2+rect[1/2]);
            light1.lenth=box1.size.width;
        }
//
        if (point1.y>point2.y){
            light1.top=point2;
            light1.bottom=point1;
        } else{
            light1.top=point1;
            light1.bottom=point2;
        }

        light1.bottom=2*light1.bottom-light1.centre;
        light1.top=2*light1.top-light1.centre;

        Light_list.push_back(light1);
    }

    Armour armour1;
    for (int i = 0; i < Light_list.size()-1; ++i) {
        for (int j = i+1; j <Light_list.size()-1 ; ++j) {
            Light_bar light1,light2;
            light1=Light_list[i];
            light2=Light_list[j];
//边长比
            float Length_ratio=light1.lenth/light2.lenth;
            if (Length_ratio<0.5 && Length_ratio>1.5){
                break;
            }
//中心距
            float Distance_centers= sqrt(pow(abs(light1.centre.x-light2.centre.x),2)+pow(abs(light1.centre.y-light2.centre.y),2));

            if (Distance_centers>(2*light1.lenth) && Distance_centers>(2*light2.lenth)){
                break;
            }
            if (light1.centre.x<=light2.centre.x){
                armour1.light_left=light1;
                armour1.light_right=light2;
            } else{
                armour1.light_left=light2;
                armour1.light_right=light1;
            }
            if ((Distance_centers*2/(light1.lenth+light2.lenth))>1){
                armour1.armour_type="LARGE";
            }
            armour1.Distance_centers=Distance_centers;
            Armour_list.push_back(armour1);
        }
    }
}

void ARMOUR::extractNumbers() {

    std::string model_path="/home/m/CLionProjects/NEXT-E-M/fc.onnx";
    // Light length in image
    const int light_length = 12;
    // Image size after warp
    const int warp_height = 28;
    const int small_armor_width = 32;
    const int large_armor_width = 54;
    // Number ROI size
    const cv::Size roi_size(28, 20);

    if (Armour_list.size()==0){
        std::cout<<"\nno Armor\n";
    } else{
        for (int i = 0; i < Armour_list.size(); ++i) {
            Armour armour1=Armour_list[i];

            cv::Point2f lights_vertices[4]= {
                    armour1.light_left.bottom,
                    armour1.light_left.top,
                    armour1.light_right.top,
                    armour1.light_right.bottom,
            };
            cv::line(light_img,armour1.light_left.bottom,armour1.light_left.top,cv::Scalar(0,255,0),1,1);
            cv::line(light_img,armour1.light_right.bottom,armour1.light_right.top,cv::Scalar(0,255,0),1,1);

            const int top_light_y = (warp_height - light_length) / 2 - 1;
            const int bottom_light_y = top_light_y + light_length;
            const int warp_width = armour1.armour_type == "SMALL" ? small_armor_width : large_armor_width;
            cv::Point2f target_vertices[4] = {
                    cv::Point(0, bottom_light_y),
                    cv::Point(0, top_light_y),
                    cv::Point(warp_width - 1, top_light_y),
                    cv::Point(warp_width - 1, bottom_light_y),
            };
            cv::Mat number_image;
            auto rotation_matrix = cv::getPerspectiveTransform(lights_vertices, target_vertices);

            cv::warpPerspective(Cap_img, number_image, rotation_matrix, cv::Size(warp_width, warp_height));

            number_image = number_image(cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));

            // Binarize
            cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);
            cv::Scalar tempVal = cv::mean( number_image );
            cv::threshold(number_image, number_image, tempVal.val[0], 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

            number_image /=255.0;

            // 将图像转换成dnn可接受的格式
            cv::Mat blob;
            cv::dnn::blobFromImage(number_image, blob, 1., cv::Size(28, 20));

            cv::dnn::Net net_ = cv::dnn::readNetFromONNX(model_path);
            // 将图像blob传入神经网络
            net_.setInput(blob);
            // 通过神经网络进行前向传递
            cv::Mat outputs = net_.forward();

            // 对输出结果进行softmax计算
            float max_prob = *std::max_element(outputs.begin<float>(), outputs.end<float>());
            cv::Mat softmax_prob;
            cv::exp(outputs - max_prob, softmax_prob);
            float sum = static_cast<float>(cv::sum(softmax_prob)[0]);
            softmax_prob /= sum;

            // 找到最大概率值及其对应的分类id
            double confidence;
            cv::Point class_id_point;
            minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
            int label_id = class_id_point.x;
            if (label_id==8){
                Armour_list.erase(Armour_list.begin()+i);
                i--;
                continue;
            }
            Armour_list[i].lable_id=class_names_[label_id];
            Armour_list[i].confidence=confidence;
        }
    }
}

void ARMOUR::distance_measurement() {

    for (int i = 0; i < Armour_list.size(); ++i){
        Armour armour1=Armour_list[i];
        std::vector<cv::Point3d> model_points;
        if (armour1.armour_type=="LARGE") {
            model_points.push_back(cv::Point3d(-115.0f, -63.5f, 0)); // 左上角顺时针
            model_points.push_back(cv::Point3d(+115.0f,-63.5f, 0));
            model_points.push_back(cv::Point3d(+115.0f, +63.5f, 0));
            model_points.push_back(cv::Point3d(-115.0f, +63.5f, 0));
        }
        if (armour1.armour_type=="SMALL"){
            model_points.push_back(cv::Point3d(-67.5f, -62.5f, 0)); // 左上角顺时针
            model_points.push_back(cv::Point3d(+67.5f, -62.5f, 0));
            model_points.push_back(cv::Point3d(+67.5f, +62.5f, 0));
            model_points.push_back(cv::Point3d(-67.5f, +62.5f, 0));
        }
//        cout<<armour1.left_top<<endl;
        std::vector<cv::Point2d> image_points;
        image_points.push_back(cv::Point2d (armour1.light_left.top));
        image_points.push_back(cv::Point2d (armour1.light_right.top));
        image_points.push_back(cv::Point2d (armour1.light_right.bottom));
        image_points.push_back(cv::Point2d (armour1.light_left.bottom));
//        cout<<image_points<<"image_points"<<endl<<model_points<<"model_points"<<endl;
        cv::Mat translation_vector;
        cv::Mat rotation_vector;
//        cout<<"pnp"<<endl;
        // pnp求解
        cv::solvePnP(model_points, image_points, camera_matrix, dist_coeffs,
                 rotation_vector, translation_vector, 0,cv::SOLVEPNP_AP3P);
//        cout<<"rotation_vector"<<rotation_vector<<endl<<endl<<"translation_vector"<<translation_vector<<endl<<endl;

        cv::Mat Rvec;
        cv::Mat_<float> Tvec;
        rotation_vector.convertTo(Rvec, CV_32F);  // 旋转向量转换格式
        translation_vector.convertTo(Tvec, CV_32F); // 平移向量转换格式

        cv::Mat_<float> rotMat(3, 3);
        Rodrigues(Rvec, rotMat);
        // 旋转向量转成旋转矩阵
//        cout << "rotMat" << endl << rotMat << endl << endl;

        cv::Mat P_oc;
        P_oc = rotMat.inv() * Tvec;
        // 求解相机的世界坐标，得出p_oc的第三个元素即相机到物体的距离即深度信息，单位是mm
//        cout << "disance" << endl << P_oc.at<float>(0,0)<< endl;
        armour1.real_distance=sqrt(pow(P_oc.at<float>(0,0),2)+pow(P_oc.at<float>(0,1),2)+pow(P_oc.at<float>(0,2),2));

        cv::line(light_img,armour1.light_right.top,armour1.light_right.bottom,cv::Scalar(0,0,255),1,1);
        cv::line(light_img,armour1.light_left.top,armour1.light_left.bottom,cv::Scalar(0,0,255),1,1);



        putText(light_img, std::to_string(armour1.real_distance),armour1.center,1,1,cv::Scalar(0,255,0),1,1);

//        cout<<"distance "<<armour1.real_distance<<endl;
        Armour_list[i].real_distance=armour1.real_distance;
    }
}

