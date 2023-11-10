#include "include/SERIAL.h"
#include "include/ARMOUR.h"
#include "include/ROBOT.h"
#include "include/Kalman_filtering.h"
#include "hik_camera/include/HikCam.hpp"
#include "thread"
#include <time.h>

HikCam cap;

cv::Mat img;

ARMOUR armour1;

ROBOT robot1;

SERIAL serial1;

KalmanFilter KF1(6,2,0);


int stateSize = 6;
int measSize = 2;
int controlSize = 0;

Eigen::MatrixXd B(0,0);
Eigen::MatrixXd H(measSize, stateSize);
//cout << H;
Eigen::MatrixXd P(stateSize, stateSize);

Eigen::MatrixXd R(measSize, measSize);

Eigen::MatrixXd Q(stateSize, stateSize);

Eigen::VectorXd x(stateSize);
Eigen::VectorXd u(0);
Eigen::VectorXd z(measSize);

Eigen::VectorXd res(stateSize);



//串口读写

void Serial_write_thread(){
    while (true){
        serial1.serial_data.Yaw_angle=0;
        serial1.serial_data.Pitch_angle=0;
        serial1.serial_data.Fire_control="N";
        serial1.serial_data=robot1.serial_data1;
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
    cap.SetExposureTime(2000);
    cap.SetFrameRate(120);
    cap.SetStreamOn();
    printf("[CamFPS] %.1fhz\n", cap.GetFrameRate());//输出实际帧率
    while (true){
        cap.GetMat(img);
        cv::Mat srcCopy = cv::Mat(img.rows, img.cols, img.depth());
        cv::flip(img, srcCopy, -1);
        img = srcCopy;
//        cv::imshow("0",img);
//        cv::waitKey(10);
    }
}

float T=0;
void Img_processing(){
    cv::VideoCapture video("/home/m/CLionProjects/untitled/video_test/test.avi");
    while (true){

        clock_t start,end;//定义clock_t变量
        start = clock();  //开始时间

        video.read(img);
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

            armour1.extractNumbers();
            armour1.distance_measurement();
//        std::cout<<"began Image_processing";

            robot1.Armour_list=armour1.Armour_list;
            if (robot1.Armour_list.size()!=0){
                robot1.Target_selection();
                robot1.Coordinate_transformation();


//                KF1.x << robot1.P_oc.at<float>(0,0),robot1.P_oc.at<float>(0,1),robot1.P_oc.at<float>(0,2),KF1.x[3],KF1.x[4],KF1.x[5];
//                KF1.x=KF1.predict(T);
//                KF1.update(H,z);
//
//                robot1.P_oc.at<float>(0,0)=KF1.x[0];
//                robot1.P_oc.at<float>(0,1)=KF1.x[1];
//                robot1.P_oc.at<float>(0,2)=KF1.x[2];

                robot1.Trajectory_solution();

                serial1.serial_data=robot1.serial_data1;
            }
        }


        std::cout<<"pitch"<<serial1.serial_data.Pitch_angle<<"yaw"<<serial1.serial_data.Yaw_angle;
        cv::namedWindow("1", cv::WINDOW_NORMAL);
        cv::resizeWindow("1", 600, 400);
        cv::imshow("1",armour1.light_img);


        cv::namedWindow("0", cv::WINDOW_NORMAL);
        cv::resizeWindow("0", 600, 400);
        cv::imshow("0",armour1.Color_processing_img);
        cv::waitKey(100);



        end = clock();   //结束时间

        std::cout<<"Frame rate"<<CLOCKS_PER_SEC/double(end-start) <<std::endl;//输出时间（单位：ｓ）
        T=CLOCKS_PER_SEC/double(end-start);
    }
}

void KF(){
    KF1.x << 0,0,0,0,0,0;
    H <<    1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0;
    P.setIdentity();
    R.setIdentity()*0.01;
    Q.setIdentity()*0.001;
    z.setZero();
}

int main(){

//
    KF();
    KF1.init(KF1.x,KF1.P,KF1.R,KF1.Q);

//    创建线程
    std::thread serial_write (Serial_write_thread);
    serial_write.detach();

    std::thread serial_read  (Serial_read_thread);
    serial_read.detach();

    std::thread read_img (Read_img);
    read_img.detach();


    std::thread img_processing (Img_processing);
    img_processing.detach();

    while (true) {}
}
