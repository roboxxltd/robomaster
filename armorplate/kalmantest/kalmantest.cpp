#include "kalmantest.h"

RM_kalmanfilter::~RM_kalmanfilter() {
    #if COUT_STATE == 1
    cout<<"stop!!!"<<endl;
    #endif
}
RM_kalmanfilter::RM_kalmanfilter(): kalman_img(3, 3) //状态向量和测量向量都是３维。板不受控制，无控制向量，为默认0维
{
    #if COUT_STATE == 1
    cout<<"start"<<endl;
    #endif
    measurement_img = Mat::zeros(3, 1, CV_64F);
    kalman_img.transitionMatrix = (Mat_ < double > (3, 3) <<
        1, runtime, runtime * runtime / 2,
        0, 1, runtime,
        0, 0, 1);
    kalman_img.processNoiseCov = (Mat_ < double > (3, 3) <<
        pow(runtime, 5) / 20, pow(runtime, 4) / 8, pow(runtime, 3) / 6, //下一步长的计算值为当前坐标加上当前速度乘以步长
        pow(runtime, 4) / 8, pow(runtime, 3) / 3, pow(runtime, 2) / 2,
        pow(runtime, 3) / 6, pow(runtime, 2) / 2, runtime);

    kalman_img.measurementMatrix = Mat_ < double > (3, 3);
    kalman_img.measurementNoiseCov = Mat_ < double > (3, 3);
    kalman_img.errorCovPost = Mat_ < double > (3, 3);
    //kalman_img.processNoiseCov = Mat_< double >(3,3);

    setIdentity(kalman_img.measurementMatrix, Scalar::all(1)); //测量值也是坐标，不需要线性的变化
    setIdentity(kalman_img.measurementNoiseCov, Scalar::all(MNC)); //测量协方差矩阵R，更大会有更慢的回归
    setIdentity(kalman_img.errorCovPost, Scalar::all(1)); //后验协方差矩阵P，方便迭代。设为无。
    //setIdentity(kalman_img.processNoiseCov,Scalar::all(1e-5));

    kalman_img.statePost = (Mat_ < double > (3, 1) << 0, 0, 0); //后验更新值(迭代起点)
    last_point.x = 320;
    last_point.y = 240;
}
Point2f RM_kalmanfilter::point_Predict(double init_runtime, Point2d current_point)
{
    runtime = init_runtime;
    double v = (current_point.x - last_point.x) / runtime;
    double acc = (v - last_v) / runtime;

    #if COUT_STATE == 1
    cout<<"["<<runtime<<","<<v<<","<<a<<"]"<<endl;
    #endif

    measurement_img.at < double > (0, 0) = current_point.x;
    measurement_img.at < double > (1, 0) = v;
    measurement_img.at < double > (2, 0) = acc;

    Mat prediction2 = kalman_img.predict(); //至此完成了对下一帧单纯计算的预测，得出的点更加平稳。如果保证测量值够准，可以直接取这里算出的点
    Mat prediction = kalman_img.correct(measurement_img); //至此完成了对下一帧的最优估计，得出的点理论上更接近真实值。同时用于迭代，得出statePost供下一帧predict计算
    Point2f temp_point = Point2f(prediction.at < double > (0, 0), current_point.y);

    double temp_x = current_point.x + pow(ANTI_RANGE,n*0.8) * (current_point.x - temp_point.x);
    Point2f anti_kalman_point;

    if(acc<-ACC||acc>ACC)
    {
        n++;
    }
    else{n=2;}
    if(fabs(v-last_v)==fabs(v)+fabs(last_v))
    {
        temp_x=current_point.x;
    }//如果目标点是在左右摆就不预测
    if(temp_x <= SIZE_X && temp_x >= 0)
    {
        if (abs(current_point.x - temp_point.x) > DEAD_BAND)
        {
            anti_kalman_point.x = temp_x;
        }
        else
        {
            anti_kalman_point.x = current_point.x;
        }
    }
    else
    {
        anti_kalman_point.x = current_point.x;
    }


    anti_kalman_point.y = temp_point.y;

    last_v = v;
    last_point = current_point;
// #ifdef PID
//     anti_kalman_point = pid_Control_predictor(anti_kalman_point,current_point);
// #endif
    return anti_kalman_point;
}
//
float RM_kalmanfilter::point_dis(Point p1, Point p2)
{
    float D = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
    return D;
}


// Point RM_kalmanfilter::pid_Control_predictor(Point predict, Point local)
// {
//     cv::Point true_location;  // 返回的坐标点，后续可能会改成角度，暂留

//     int error_x;  // x坐标之间的差值
//     int error_y;  // y坐标之间的差值
//     int dis;

//     predict_x = predict.x;  /*预测位置*/
//     predict_y = predict.y;

//     last_x = local.x;  /*上一次的实际位置*/
//     last_y = local.y;

//     error_x = abs(predict_x - last_x);
//     error_y = abs(predict_y - last_y);

//     dis = point_dis(predict, local);


//     if(dis < 3)
//     { // 差值过小的不进入PID处理
//         true_location.x = last_x;  // 差值过小就不用预测位，防止云台抖动
//         true_location.y = last_y;
//     }


//     if(dis > 5 && dis < 10)
//     {  // 合理范围内就用预测点，具体参数要依实际调整
//         true_location.x = predict_x;
//         true_location.y = predict_y;
//     }


//     if(dis > 10)  // 依实际调整
//     {
//         // 调用PID //
//         error_x = error_x*KP + error_x*KI + error_x*KD;  // PID修正误差
//         error_y = error_y*KP + error_y*KI + error_y*KD;

//         /*下面有四种情况，直接看代码理解会比较好，注意图像坐标是左上角为0点*/
//         if(predict_x >= last_x && predict_y >= last_y)
//         {
//             true_location.x = last_x + error_x;
//             true_location.y = last_y + error_y;
//         }
//         else if (predict_x >= last_x && predict_y <= last_y)
//         {
//             true_location.x = last_x + error_x;
//             true_location.y = last_y - error_y;
//         }
//         else if (predict_x <= last_x && predict_y <= last_y)
//         {
//             true_location.x = last_x - error_x;
//             true_location.y = last_y - error_y;
//         }
//         else if (predict_x <= last_x && predict_y >= last_y)
//         {
//             true_location.x = last_x - error_x;
//             true_location.y = last_y + error_y;
//         }
//         else {

//         }

//      }
//     return true_location;  // 返回最终修复的坐标
// }
