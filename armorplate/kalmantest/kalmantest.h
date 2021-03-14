#ifndef KALMANTEST_H
#define KALMANTEST_H

#include "configure.h"
// #include "serialport.h"
#include "control.h"

class RM_kalmanfilter
{
public:
    RM_kalmanfilter();
    ~RM_kalmanfilter();
    Point2f point_Predict(double g_runtime,Point2d current_point);

    //pid修正
    float point_dis(Point p1, Point p2);  // 两点距离
    Point pid_Control_predictor(Point predict, Point local);  // PID控制,返回的是一个修正后的点

private:
    Mat measurement_img;//测量矩阵
    cv::KalmanFilter kalman_img;
    Point2f last_point;
//    Point2f last_predic_point;
    double runtime=(1e-2) + 0.005666666f;
    double last_v=0;
    int n=2;

    //pid修正
    int predict_x;  // 预测位x
    int predict_y;  // 预测位y

    int last_x;  // 当前装甲板x
    int last_y;  // 当前装甲板y

    int true_x;  // 修正后的x
    int true_y;  // 修正后的y

};

#endif // KALMANTEST_H
