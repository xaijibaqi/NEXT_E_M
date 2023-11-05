#include "include/SERIAL.h"
#include "include/ARMOUR.h"
#include "include/ROBOT.h"
#include "hik_camera/include/HikCam.hpp"
#include "thread"


HikCam cap;
cv::Mat img;

ARMOUR armour1;

ROBOT robot1;

SERIAL serial1;

//串口读写

void Serial_write_thread(){
    while (true){



        serial1.send();
    }
}
void Serial_read_thread(){
    while (true){
        serial1.receive();
        robot1.Enemy_color=serial1.robot1.Enemy_color;
        robot1.Yaw_angle=serial1.robot1.Yaw_angle;
        robot1.Pitch_angle=serial1.robot1.Pitch_angle;
    }
}

//机器人

//相机初始化

void Read_img(){
    cap.StartDevice(0);
    cap.SetResolution(1280,1024);
    cap.SetPixelFormat(17301514);
    cap.SetExposureTime(5000);
    cap.SetFrameRate(120);
    cap.SetStreamOn();
    printf("[CamFPS] %.1fhz\n", cap.GetFrameRate());//输出实际帧率
    while (true){
        cap.GetMat(img);
//        cv::imshow("0",img);
//        cv::waitKey(10);

    }
}

//void Read_video(){
//    cv::VideoCapture video("/home/m/CLionProjects/untitled/video_test/test.avi");
//    while (true)
//}

void Img_processing(){
//    cv::VideoCapture video("/home/m/CLionProjects/untitled/video_test/test.avi");
    while (true){

//        video.read(img);

        while (img.empty()){
//            std::cout<<"no img\n";
            continue;
        }

        armour1.Cap_img=img.clone();
        armour1.Enemy_color=serial1.robot1.Enemy_color;
        armour1.Image_processing();
        armour1.find_light();
        if (armour1.Armour_list.size()!=0){
            serial1.serial_data.Pitch_angle=0;
            serial1.serial_data.Yaw_angle=0;
            serial1.serial_data.Fire_control="N";
//            std::cout<<"\n no Aromo";
//            cv::namedWindow("1", cv::WINDOW_NORMAL);
//            cv::resizeWindow("1", 600, 400);
//            cv::imshow("1",armour1.light_img);

//
//            cv::namedWindow("0", cv::WINDOW_NORMAL);
//            cv::resizeWindow("0", 600, 400);
//            cv::imshow("0",armour1.Color_processing_img);
//            cv::waitKey(100);
            armour1.extractNumbers();
            armour1.distance_measurement();
//        std::cout<<"began Image_processing";

            robot1.Armour_list=armour1.Armour_list;
            if (robot1.Armour_list.size()!=0){
                robot1.Target_selection();
                robot1.Coordinate_transformation();
                robot1.Trajectory_solution();

                serial1.serial_data=robot1.serial_data1;
            }
        }



        cv::namedWindow("1", cv::WINDOW_NORMAL);
        cv::resizeWindow("1", 600, 400);
        cv::imshow("1",armour1.light_img);


        cv::namedWindow("0", cv::WINDOW_NORMAL);
        cv::resizeWindow("0", 600, 400);
        cv::imshow("0",armour1.Color_processing_img);
        cv::waitKey(100);


//        std::cout<<"\nRobot "<<robot1.Pitch_angle<<"\n";
//        std::cout<<"\ndadsa d "<<robot1.Yaw_angle<<"\n";
    }
}



int main(){
//    创建线程
    std::thread serial_write (Serial_write_thread);
    serial_write.detach();

    std::thread serial_read  (Serial_read_thread);
    serial_read.detach();

    std::thread read_img (Read_img);
    read_img.detach();

//    while (true){
//        Img_processing();
//    }
    std::thread img_processing (Img_processing);
    img_processing.detach();

    while (true) {
//        if (armour1.light_img.empty()){
//            continue;
//        }
//
    }
}
