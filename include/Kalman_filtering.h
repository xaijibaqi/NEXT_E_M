//
// Created by m on 23-11-11.
//

#ifndef NEXT_E_M_KALMAN_FILTERING_H
#define NEXT_E_M_KALMAN_FILTERING_H


#include <Eigen/Dense>
#include "iostream"


class KalmanFilter
{
public:
    int stateSize; //state variable's dimenssion
    int measSize; //measurement variable's dimession
    int uSize; //control variables's dimenssion
    Eigen::VectorXd x;
    Eigen::VectorXd z;
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::VectorXd u;
    Eigen::MatrixXd P;//coveriance
    Eigen::MatrixXd H;
    Eigen::MatrixXd R;//measurement noise covariance
    Eigen::MatrixXd Q;//process noise covariance
    KalmanFilter(int stateSize_, int measSize_,int uSize_);
    ~KalmanFilter(){}
    void init(Eigen::VectorXd &x_, Eigen::MatrixXd& P_,Eigen::MatrixXd& R_, Eigen::MatrixXd& Q_);
    Eigen::VectorXd predict(float T);
    Eigen::VectorXd predict(Eigen::MatrixXd& A_, Eigen::MatrixXd &B_, Eigen::VectorXd &u_);
    void update(Eigen::MatrixXd& H_,Eigen::VectorXd z_meas);
};


#endif //NEXT_E_M_KALMAN_FILTERING_H
