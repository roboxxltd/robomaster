#include "configure.h"
#include "control.h"

class Max_Buff
{
public:
    Max_Buff() {}
    ~Max_Buff() {}
    struct buff_target
    {
        bool activation = false;
        Point position;
        RotatedRect rect;
    };
    //预处理
    void pretreat(Mat frame, int enemy_color);
    //找大神符中心的R
    bool Looking_for_center();
    //寻找目标
    int Looking_for_target();
    //计算坐标
    void Calculating_coordinates(int i);
    int average_color(Mat roi);
    Mat max_buff_roi(int i);
    float Distance(Point a, Point b)
    {
        return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    }
    double Cross(Point a, Point b, Point c)
    {
        return ((b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x));
    }
    int Getstate();

    vector<buff_target> buff;
    vector<Point> armor_center;
    vector<RotatedRect> max_buff_rects;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    Mat frame;
    Mat src_img;
    Mat mask;
    Mat gray_img;

    Rect roi;
    RotatedRect rects_2d;
    Point2f pre_center;
    Point2f calculation_position[2];
    Point2f central_point;

    float radius, small_radius;
    float forecast_angle = 0;
    float angle_cos, angle_sin, angle;
    float buff_angle_ = 0;
    float diff_angle_ = 0;
    float last_angle = 0;
    float d_angle_ = 1; //0

    bool central_success = false;
    bool choice_success = false;
    bool roi_success = false;

    //蓝色th参数
    int blue_armor_gray_th = 70;
    int blue_armor_color_th = 45;
    //红色th参数
    int red_armor_gray_th = 80;
    int red_armor_color_th = 45;
    int input = 0;

    float yaw_data = 640;   //云台偏航
    int _yaw_data = 400;    //偏航值的正负
    float pitch_data = 0; //云台俯仰
    int _pitch_data = 0;  //俯仰值的正负
    int depth = 0;        //深度 -- 装甲板距离相机的距离
    int hit_subscript = 0;
    int make_up_angle = 30;
    int find_cnt_ = 0;
    int direction_tmp_ = 0;

    //弹道补偿
    int offset_x = 550;        //小：153
    int offset_y = 100;         //小： 30
    int _offset_x = 0;         //正1 负0 小：0
    int _offset_y = 0;         //正1 负0 小：1
    int offset_amplitude = 12; // 0~50 倍 20
    int offset_excursion = 31; //数值除以十 1~10 精度为0.1 10
    int offset_preangle = PRE_ANGLE;
    int offset_ratio = 2; //0~5° 精度0.1 1.5感觉最好
};

struct buff_target
{
    bool activation = false;
    Point position;
    RotatedRect rect;
};