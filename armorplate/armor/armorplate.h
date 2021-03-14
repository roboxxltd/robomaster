#include "control.h"
#include "configure.h"

using namespace cv;
using namespace std;

/**
 * @brief 装甲板识别模块的起点
 * 
 */
class ArmorPlate
{
public:
    // Mat draw_img = Mat::zeros(Size(CAMERA_RESOLUTION_COLS, CAMERA_RESOLUTION_ROWS), CV_8UC3);
    int rect_num = 0;
    int lost = 0; //丢失帧数
    int _yaw = 0;
    float yaw = 0;
    float pitch = 0;
    int _pitch = 0;
    int depth = 0;
    int data_type = 0;
    int is_shooting = 0;

    //弹道补偿
    int offset_x = 0;  //小：153
    int offset_y = 0;     //小： 30
    int _offset_x = 1;    //正1 负0 小：0
    int _offset_y = 0;    //正1 负0 小：1
    int offset_ratio = 15; //0~5° 精度0.1 1.5感觉最好
    Rect armor_roi;
    bool lost_success_armor = false; //上一帧是否找到装甲板
    bool success_armor = false;      //这一帧是否找到装甲板
    /*change place */
    int char_armor = 0; //判断打击装甲板的种类
    /*********/
    void eliminate();
    void run();
    ArmorPlate() {}
    ~ArmorPlate() {}
};

/**
 * @brief 识别灯条
 * 
 */
class LightBar : public ArmorPlate
{
public:
    vector<RotatedRect> light;       //储存灯条的旋转矩形
    vector<RotatedRect> armor;       //储存装甲板的旋转矩形
    RotatedRect roi_rect;            //下次图像ROI位置
    vector<int> light_subscript;     //灯条下标
    vector<bool> priority;           //优先级
    
    float light_aspect_ratio = 0.8f; //灯条长宽比
    float light_angle = 40.0;        //灯条角度
    RotatedRect lost_armor;          //上一帧装甲板位置
    bool armor_fitting(Mat src_img);
    bool light_judge(int i, int j);
    bool find_light(Mat mask);
    int average_color(Mat roi);
    void eliminate();
    int optimal_armor();
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
    int blue_armor_color_th = 100;
    //红色th参数
    int red_armor_gray_th = 40; //视频20
    int red_armor_color_th = 70;
};
