#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include <vector>

/**
 * @brief 滑动条开启
 * @param 0 关闭
 * @param 1 开启
 */
#define SLIDE_BAR_ON 1
/**
 * @brief 击打判断
 * @param 0 击打离中心点最小装甲板
 * @param 1 击打最大装甲板
 */
#define STRIKE_JUDGMENT 0

/**
 * @brief 绘画
 * @param 0 不绘画
 * @param 1 绘画
 */
#define DRAWIMG 1;

using namespace cv;
using namespace std;

class armorplate
{
private:
    /*储存处理后的灯条*/
    vector<RotatedRect> light;
    /*黑板*/
    Mat draw_img;
    vector<vector<Point>> contours;
    Rect armor_roi;
    bool success_armor = false;
    vector<int> left; //灯条下标
    vector<int> right;
    Point Center;            //装甲板中心点
    Point lost_Center;       //上一帧的击打位置
    int armor_num = 0;       //装甲板数量
    int big_small_armor = 0; //大于4识别在不丢三帧前提下识别大装甲板
    Rect armor_rects;
    float speed_x; //装甲板水平方向速度
    float speed_y; //装甲板数值方向速度
    vector<bool> armor_level;

public:
    armorplate();
    ~armorplate();
    /*预处理*/
    void Pretreatment();
    /*灯条判断*/
    bool Light_judge(int i, int j);
    /*击打相对位置判断*/
    void Strike_max();
    /*计算两点之间的距离*/
    double Distance(Point a, Point b)
    {
        return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    }
    //计算图片平均强度
    int Average_color(Mat roi);
    //计算是否丢失
    bool Lost_target();
    //释放缓存
    void Eliminate();

    bool Armor_judge();

    void Coordinate_transformation();
};
