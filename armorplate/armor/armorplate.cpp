#include "armorplate.h"

/**
 * @brief 求两点之间的距离
 * 
 * @param a 点A
 * @param b 点B
 * @return double 两点之间的距离 
 */
float Distance(Point a, Point b)
{
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

void ArmorPlate::eliminate()
{
    this->success_armor = false;
    // this->draw_img = Mat::zeros(Size(CAMERA_RESOLUTION_COLS, CAMERA_RESOLUTION_ROWS), CV_8UC3);
    this->rect_num = 0;
#if ROI_IMG == 1

    // if (armor_roi.x <= 0 || armor_roi.x >= CAMERA_RESOLUTION_COLS || armor_roi.y <= 0 || armor_roi.y >= CAMERA_RESOLUTION_ROWS ||
    // armor_roi.width < 0 || armor_roi.width > CAMERA_RESOLUTION_COLS || armor_roi.height > CAMERA_RESOLUTION_ROWS ||armor_roi.height<0)
    // {
    //     lost_success_armor = false;
    //     armor_roi = Rect(0, 0, 0, 0);
    // }

#endif
}

/**
 * @brief 图像预处理
 * 
 * @param src_img -传入原图像
 * @param enemy_color -传入敌方颜色
 */
void ImageProcess::pretreat(Mat src_img, int enemy_color)
{
    //保存原图像
    this->frame = src_img;
    //转灰度图
    Mat gray_img;
    cvtColor(this->frame, gray_img, COLOR_BGR2GRAY);
    //分离通道
    vector<Mat> _split;
    split(src_img, _split);
    //判断颜色
    Mat bin_img_color, bin_img_gray;
    
    
    if (enemy_color == 0)
    {
        
        subtract(_split[0], _split[2], bin_img_color); // b - r

#if IS_PARAM_ADJUSTMENT == 1
        namedWindow("src_img", WINDOW_AUTOSIZE);
        createTrackbar("GRAY_TH_BLUE:", "src_img", &blue_armor_gray_th, 255, NULL);
        createTrackbar("COLOR_TH_BLUE:", "src_img", &blue_armor_color_th, 255, NULL);
        threshold(gray_img, bin_img_gray, this->blue_armor_gray_th, 255, THRESH_BINARY);
        threshold(bin_img_color, bin_img_color, this->blue_armor_color_th, 255, THRESH_BINARY);
#else
        threshold(gray_img, bin_img_gray, blue_armor_gray_th, 255, THRESH_BINARY);
        threshold(bin_img_color, bin_img_color, blue_armor_color_th, 255, THRESH_BINARY);
#endif

    }
    else if (enemy_color == 1)
    {

        subtract(_split[2], _split[0], bin_img_color); // r - b
        double t = (double)cv::getTickCount(); //开始计时
#if IS_PARAM_ADJUSTMENT == 1
        namedWindow("src_img", WINDOW_AUTOSIZE);
        createTrackbar("GRAY_TH_RED:", "src_img", &red_armor_gray_th, 255);
        createTrackbar("COLOR_TH_RED:", "src_img", &red_armor_color_th, 255);
        threshold(gray_img, bin_img_gray, this->red_armor_gray_th, 255, THRESH_BINARY);
        threshold(bin_img_color, bin_img_color, this->red_armor_color_th, 255, THRESH_BINARY);
#else
        threshold(gray_img, bin_img_gray, red_armor_gray_th, 255, THRESH_BINARY);
        threshold(bin_img_color, bin_img_color, red_armor_color_th, 255, THRESH_BINARY);
#endif
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency(); //结束计时
        int fps = int(1.0 / t);                                        //转换为帧率
        // cout << "FPS: " << fps << endl;                                //输出帧率
    }

    Mat element = getStructuringElement(MORPH_ELLIPSE, cv::Size(3, 5));
#if SHOW_BIN_IMG == 1
    imshow("gray_img", bin_img_gray);
    imshow("mask", bin_img_color);
#endif

    // bitwise_and(bin_img_color, bin_img_gray, bin_img_color);
    // medianBlur(bin_img_color, bin_img_color, 5);
    dilate(bin_img_gray, bin_img_gray, element);
    dilate(bin_img_color, bin_img_color, element);
    medianBlur(bin_img_color, bin_img_color, 3);

    bitwise_and(bin_img_color, bin_img_gray, bin_img_color);
    // dilate(bin_img_color, bin_img_color, element);
#if SHOW_BIN_IMG == 1
    imshow("src_img", bin_img_color);
#endif
    //保存处理后的图片
    this->mask = bin_img_color;
    this->gray_img = bin_img_gray;
    _split.clear();
    vector<Mat>(_split).swap(_split);
}

/**
 * @brief 寻找可能为等灯条的物体
 * 
 * @param mask 传入预处理后的二值化图片
 * @return true 找到灯条
 * @return false 没有灯条
 */
bool LightBar::find_light(Mat mask)
{

#if DRAW_LIGHT_IMG == 1
    Mat draw = Mat::zeros(mask.size(), CV_8UC3);
#endif
    // this->img_cols = mask.cols;
    // this->img_rows = mask.rows;
    int success = 0;
    RotatedRect minRect;
    /*轮廓周长*/
    int perimeter = 0;
    vector<vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    //筛选，去除一部分矩形
    for (size_t i = 0; i < contours.size(); i++)
    {
        perimeter = arcLength(contours[i], true); //轮廓周长
        if (perimeter > 30 && perimeter < 4000 && contours[i].size() >= 5)
        {
            //椭圆拟合
            minRect = fitEllipse(Mat(contours[i]));

            //重新定义长宽和角度
            if (minRect.angle > 90.0f)
            {
                minRect.angle = minRect.angle - 180.0f;
            }

            //灯条长宽比
            float light_w_h;
            if (minRect.size.height == 0)
            {
                continue;
            }
            light_w_h = minRect.size.width / minRect.size.height;

            if (fabs(minRect.angle) < this->light_angle && light_w_h < this->light_aspect_ratio)
            {
                this->light.push_back(minRect); //保存灯条
                success++;
#if DRAW_LIGHT_IMG == 1
                Point2f vertex[4];
                minRect.points(vertex);
                for (int l = 0; l < 4; l++)
                {
                    line(draw, vertex[l], vertex[(l + 1) % 4], Scalar(0, 255, 255), 3, 8);
                }
                imshow("draw", draw);
#endif
            }
        }
    }
    contours.clear();
    vector<vector<Point>> (contours).swap(contours);
#if DRAW_LIGHT_IMG == 1
    imshow("light", draw);
#endif
    return success;
}

/**
 * @brief 灯条筛选装甲板
 * 
 * @param src_img 传入灰度图
 * @return int 返回装甲板数量
 */
bool LightBar::armor_fitting(Mat src_img)
{
    int success_armor = 0;
    for (size_t i = 0; i < light.size(); i++)
    {
        for (size_t j = i + 1; j < light.size(); j++)
        {
            //区分左右灯条
            int light_left = 0, light_right = 0;
            if (light[i].center.x > light[j].center.x)
            {
                light_left = j;
                light_right = i;
            }
            else
            {
                light_left = i;
                light_right = j;
            }

            //计算灯条中心点形成的斜率
            float error_angle = atan((light[light_right].center.y - light[light_left].center.y) / (light[light_right].center.x - light[light_left].center.x));

            if (error_angle < 9.0f)
            {
                if (this->light_judge(light_left, light_right))
                {
                    if (this->average_color(this->armor_rect(light_left, light_right, src_img, error_angle)) < 50)
                    {
                        success_armor++;
                        //储存灯条左右下标
                        this->light_subscript.push_back(light_left);
                        this->light_subscript.push_back(light_right);
                    }
                }
            }
        }
    }
    if (success_armor > 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @brief 寻找可能为装甲板的位置
 * @param i 左light的下标
 * @param j 右light的下标
 * @return true 找到了符合装甲板条件的位置
 * @return false 没找到了符合装甲板条件的位置
 */
bool LightBar::light_judge(int i, int j)
{
    float left_h = MAX(light[i].size.height, light[i].size.width);
    float left_w = MIN(light[i].size.height, light[i].size.width);
    float right_h = MAX(light[j].size.height, light[j].size.width);
    float right_w = MIN(light[j].size.height, light[j].size.width);
    // cout<<"111111111111111111111111111111111111"<<endl;
    if (left_h < right_h * 1.7 && left_w > right_w * 0.5 && left_h > right_h * 0.5 && left_w < right_w * 1.7)
    {
        // cout<<"22222222222222222222222222222222222222"<<endl;
        // float h_ = MIN(left_h , right_h) / 2.0f;
        float h_max = (left_h + right_h) / 2.0f;
        // 两个灯条高度差不大
        // cout<<fabs(light[i].center.y - light[j].center.y)<<endl;
        if (fabs(light[i].center.y - light[j].center.y) < h_max * 1.6)
        {
            
            //装甲板长宽比
            float w_max = Distance(light[j].center , light[i].center);
            // float h_max = (left_h + right_h) / 2.0f;
            // cout<<w_max/h_max<<endl;
            if (w_max < h_max * 2.5 && w_max > h_max * 0.5f)
            {
                /*change place*/
                char_armor = 1;
                // 100;
                /*************/
                return true;
            }

            if (w_max > h_max * 3.05f && w_max < h_max * 6.f)
            {
                /*change place*/
                char_armor = 2;
                return true;
            }
        }
    }
    return false;
}

/**
 * @brief 计算图像中像素点的平均强度
 * 
 * @param roi 传入需要计算的图像
 * @return int 返回平均强度
 */
int LightBar::average_color(Mat roi)
{
    int average_intensity = static_cast<int>(mean(roi).val[0]);
    return average_intensity;
}

/**
 * @brief 保存装甲板的旋转矩形
 * 
 * @param i Light下标
 * @param j Light下标
 * @param src_img 传入图像
 * @param angle 旋转矩形角度
 * @return Mat 返回装甲板的ROI区域
 */
Mat LightBar::armor_rect(int i, int j, Mat src_img, float angle)
{
    float right_w = MIN(light[j].size.width, light[j].size.height);
    float right_h = MAX(light[j].size.width, light[j].size.height);
    float left_w = MIN(light[i].size.width, light[i].size.height);
    float left_h = MAX(light[i].size.width, light[i].size.height);
    // cout<<right_h<<", "<<left_h<<endl;
    RotatedRect rects = RotatedRect(
        Point((light[i].center.x + light[j].center.x) / 2, (light[i].center.y + light[j].center.y) / 2),
        Size(Distance(light[i].center, light[j].center) - (left_w + right_w), 
        (right_h + left_h)/2),
        angle);
    this->armor.push_back(rects); //储存装甲板旋转矩形
    Rect _rect = rects.boundingRect();
    Mat roi;
    if (_rect.x <= 0)
    {
        _rect.x = 0;
    }
    if (_rect.y <= 0)
    {
        _rect.y = 0;
    }
    if (_rect.y + _rect.height >= src_img.rows)
    {
        _rect.height = src_img.rows - abs(_rect.y);
    }
    if (_rect.x + _rect.width >= src_img.cols)
    {
        _rect.width = src_img.cols - abs(_rect.x);
    }

    roi = src_img(_rect);
    return roi;
}

/**
 * @brief 多个装甲板筛选优先级
 * 
 * @return Point 最优先返回最大装甲板
 */
int LightBar::optimal_armor()
{
    size_t max = 0;
    int max_num = 0;
    if (this->armor.size() <= 1)
        return 0;
    for (size_t i = 0; i < this->light_subscript.size(); i += 2)
    {
        //灯条是“\\”或者“//”和“||”这样
        // cout<<fabs(this->light[this->light_subscript[i]].angle - this->light[this->light_subscript[i + 1]].angle)<<endl;
        if (fabs(this->light[this->light_subscript[i]].angle - this->light[this->light_subscript[i + 1]].angle) < 8.0f)
        {
            this->priority.push_back(true);
        }
        else
        {
            continue;
        }

        //灯条的高度差不超过最大灯条高度的四分之一
        int left_h = MAX(this->light[this->light_subscript[i]].size.width, this->light[this->light_subscript[i]].size.height);
        int right_h = MAX(this->light[this->light_subscript[i + 1]].size.width, this->light[this->light_subscript[i + 1]].size.height);
        int _h = MAX(left_h, right_h) / 4;
        int h_ = MIN(left_h, right_h);
        if (fabs(this->light[this->light_subscript[i]].center.y - this->light[this->light_subscript[i + 1]].center.y) < h_/2)
        {
            this->priority.push_back(true);
        }

        //灯条高度差距不超过10%
        if (left_h > h_ /2 || right_h > _h /2)
        {
            this->priority.push_back(true);
        }
        else{
            continue;
        }

        //灯条中心点形成的直线与水平线的夹角f
        float delta_y = this->light[this->light_subscript[i]].center.y - this->light[this->light_subscript[i + 1]].center.y;
        float delta_x = this->light[this->light_subscript[i]].center.x - this->light[this->light_subscript[i + 1]].center.x;
        float deviationAngle = abs(atan(delta_y / delta_x)) * 180 / CV_PI;
        if (deviationAngle < 40)
        {
            this->priority.push_back(true);
        }

        //灯条的中心点到装甲板中心点的距离超过最小灯条的宽度
        if (Distance(this->armor[i / 2].center, this->light[this->light_subscript[i]].center) > h_ / 2 && Distance(this->armor[i / 2].center, this->light[this->light_subscript[i + 1]].center) > h_ / 2)
        {
            this->priority.push_back(true);
        }

        if (this->armor[i / 2].size.width / this->armor[i / 2].size.height < 1.5f)
        {
            this->priority.push_back(true);
        }
        else if (this->armor[i / 2].size.width / this->armor[i / 2].size.height >= 2)
        {
            continue;
        }

        if (this->priority.size() > 2)
        {
            if (this->priority.size() > max)
            {
                max = this->priority.size();
                max_num = i;
            }
            else if (this->priority.size() == max)
            {
                //符合程度相同时选取更近的一位
                // int this_wamh = Distance(this->light[this->light_subscript[i]].center , this->light[this->light_subscript[i + 1]].center);
                // int max_wmah = Distance(this->light[this->light_subscript[max_num]].center , this->light[this->light_subscript[max_num + 1]].center);
                int this_wamh = this->light[light_subscript[i]].size.height;
                int max_wmah = this->light[light_subscript[max_num]].size.height;
                if (this->armor[i / 2].size.width / this->armor[i / 2].size.height < 2)
                {
                    max_num = i;
                }
                else
                {
                    if (this_wamh > max_wmah)
                    {
                        max_num = i;
                    }
                }
            }
        }
        this->priority.clear();
    }
    return max_num;
}

/**
 * @brief 清除vector数据
 * 
 */
void LightBar::eliminate()
{
    light.clear();
    armor.clear();
    light_subscript.clear();
    priority.clear();
    vector<RotatedRect> (light).swap(light);
    vector<RotatedRect> (armor).swap(armor);
    vector<bool> (priority).swap(priority);
    vector<int> (light_subscript).swap(light_subscript);
}

/**
 * @brief ROI坐标转换
 * 
 * @param i 丢失次数
 */
void LightBar::coordinate_change(int i)
{
    if (i > 0 && i < MAXIMUM_LOSS)
    {
        //扩大两倍搜索范围
        this->roi_rect = RotatedRect(
            this->roi_rect.center,
            Size(this->roi_rect.size.width * 2, this->roi_rect.size.height * 2),
            this->roi_rect.angle);
    }
    else
    {
        //丢失过多取消ROI
        this->roi_rect = RotatedRect(
            Point(CAMERA_RESOLUTION_COLS / 2, CAMERA_RESOLUTION_ROWS / 2),
            Size(CAMERA_RESOLUTION_COLS, CAMERA_RESOLUTION_ROWS),
            this->roi_rect.angle);
    }
}
