#include <opencv2/opencv.hpp>
#include <iostream>
#include "control.h"
#include <math.h>

using namespace cv;
using namespace std;

/**
 * @brief 装甲板识别模块的起点
 * 
 */
class ArmorPlate
{
public:
    Mat draw_img = Mat::zeros(Size(IMG_COLS, IMG_ROWS), CV_8UC3);
    int rect_num = 0;
    int lost = 0;
    Rect armor_roi;
    bool lost_success_armor = false;
    bool success_armor = false;
    void eliminate();
    void run();
    ArmorPlate() {}
    ~ArmorPlate() {}
};

/**
 * @brief 识别灯条
 * 
 */
class LightBar
{
public:
    vector<RotatedRect> light;       //储存灯条的旋转矩形
    vector<RotatedRect> armor;       //储存装甲板的旋转矩形
    RotatedRect roi_rect;            //下次图像ROI位置
    float light_aspect_ratio = 0.6f; //灯条长宽比
    float light_angle = 45.0;        //灯条角度
    bool armor_fitting(Mat src_img);
    bool light_judge(int i, int j);
    bool find_light(Mat mask);
    int average_color(Mat roi);
    void eliminate();
    int optimal_armor(bool judge);
    int light_is_one();
    int img_cols;
    int img_rows;
    Mat armor_rect(int i, int j, Mat src_img, float angle);
    void coordinate_change(int i);
    LightBar() {}
    ~LightBar() {}
};

/**
 * @brief 图像处理
 * 
 */
class ImageProcess
{
public:
    void pretreat(Mat frame, int enemy_color);
    ImageProcess() {}
    ~ImageProcess() {}
    Mat frame;
    Mat mask;
    Mat gray_img;
    //蓝色th参数
    int blue_armor_gray_th = 80;
    int blue_armor_color_th = 135;
    //红色th参数
    int red_armor_gray_th = 225; //视频20
    int red_armor_color_th = 50;
};
