#include "armorplate.h"

/**
 * @brief 求两点之间的距离
 * 
 * @param a 点A
 * @param b 点B
 * @return double 两点之间的距离 
 */
double Distance(Point a, Point b)
{
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

/**
 * @brief 总运行函数-装甲板识别
 * 
 */
void ArmorPlate::run()
{
    VideoCapture cap(0);
    //检查是否成功打开
    ImageProcess img;
    LightBar rgb;
    if (!cap.isOpened())
    {
        cout << "打开失败" << endl;
    }

    for (;;)
    {

        double t = (double)cv::getTickCount(); //开始计时
        this->success_armor = false;
        Mat frame;
        cap.read(frame);
        Mat src_img;
        resize(frame, src_img, Size(IMG_COLS, IMG_ROWS));
#if ROI_IMG == 1
        if (this->lost_success_armor)
        {
            src_img = src_img(this->armor_roi);
        }
#endif
        //图像预处理
        img.pretreat(src_img, COLOR);
        //找到灯条后

        if (rgb.find_light(img.mask))
        {
            //装甲板大于等于1块时
            if (rgb.armor_fitting(img.gray_img))
            {
                this->rect_num = rgb.optimal_armor(this->lost_success_armor);
#if DRAW_ARMOR_IMG == 1
                rectangle(this->draw_img, rgb.armor[this->rect_num].boundingRect(), Scalar(0, 255, 0), 3, 8);
                rectangle(this->draw_img, rgb.roi_rect.boundingRect(), Scalar(255, 200, 0), 3, 8);
                rectangle(src_img, rgb.armor[this->rect_num].boundingRect(), Scalar(0, 255, 0), 3, 8);
                rectangle(src_img, rgb.roi_rect.boundingRect(), Scalar(255, 200, 0), 3, 8);
#endif
                this->success_armor = true;
            }
            else //丢失目标
            {

                //扩大搜索目标
#if ROI_IMG == 1
                this->lost++;
                rgb.roi_rect = RotatedRect(
                    Point(IMG_COLS / 2, IMG_ROWS / 2),
                    Size(IMG_COLS, IMG_ROWS),
                    rgb.roi_rect.angle);
#endif
            }
        }

#if ROI_IMG == 1
        this->armor_roi = rgb.roi_rect.boundingRect(); //保存roi位置
#endif
        rgb.eliminate();
        this->eliminate();

        imshow("frame", src_img);
        imshow("draw", this->draw_img);

        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency(); //结束计时
        int fps = int(1.0 / t);                                        //转换为帧率
        cout << "FPS: " << fps << endl;                                //输出帧率

        char c = waitKey(1);
        if (c == 27) //"Esc"-退出
        {
            break;
        }
    }
}

void ArmorPlate::eliminate()
{
    this->draw_img = Mat::zeros(Size(IMG_COLS, IMG_ROWS), CV_8UC3);
    this->rect_num = 0;
    this->lost_success_armor = this->success_armor; //保存上一帧的参数
    this->success_armor = false;
#if ROI_IMG == 1
    if (this->armor_roi.x + this->armor_roi.width > IMG_COLS || this->armor_roi.y + this->armor_roi.height > IMG_ROWS)
    {
        this->armor_roi = Rect(0, 0, 640, 480);
    }
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
    cvtColor(src_img, gray_img, COLOR_BGR2GRAY);
    //分离通道
    vector<Mat> _split;
    split(src_img, _split);
    //判断颜色
    Mat bin_img_color, bin_img_gray;
    namedWindow("src_img", WINDOW_AUTOSIZE);
    if (enemy_color == 0)
    {
        subtract(_split[0], _split[2], bin_img_color); // b - r
#if IS_PARAM_ADJUSTMENT == 1
        createTrackbar("GRAY_TH_BLUE:", "src_img", &this->blue_armor_gray_th, 255, NULL);
        createTrackbar("COLOR_TH_BLUE:", "src_img", &this->blue_armor_color_th, 255, NULL);
        threshold(gray_img, bin_img_gray, this->blue_armor_gray_th, 255, THRESH_BINARY);
        threshold(bin_img_color, bin_img_color, this->blue_armor_color_th, 255, THRESH_BINARY);
#elif IS_PARAM_ADJUSTMENT == 0
        threshold(gray_img, bin_img_gray, blue_armor_gray_th, 255, THRESH_BINARY);
        threshold(bin_img_color, bin_img_color, blue_armor_color_th, 255, THRESH_BINARY);
#endif
    }
    else if (enemy_color == 1)
    {
        subtract(_split[2], _split[0], bin_img_color); // r - b
#if IS_PARAM_ADJUSTMENT == 1
        createTrackbar("GRAY_TH_BLUE:", "src_img", &this->red_armor_gray_th, 255);
        createTrackbar("COLOR_TH_BLUE:", "src_img", &this->red_armor_color_th, 255);
        threshold(gray_img, bin_img_gray, this->red_armor_gray_th, 255, THRESH_BINARY);
        threshold(bin_img_color, bin_img_color, this->red_armor_color_th, 255, THRESH_BINARY);
#elif IS_PARAM_ADJUSTMENT == 0
        threshold(gray_img, bin_img_gray, red_armor_gray_th, 255, THRESH_BINARY);
        threshold(bin_img_color, bin_img_color, red_armor_color_th, 255, THRESH_BINARY);
#endif
    }
    Mat element = getStructuringElement(MORPH_ELLIPSE, cv::Size(5, 9));
#if SHOW_BIN_IMG == 1
    imshow("gray_img", bin_img_gray);
    imshow("mask", bin_img_color);
#endif
    bitwise_and(bin_img_color, bin_img_gray, bin_img_color);
    medianBlur(bin_img_color, bin_img_color, 3);
    dilate(bin_img_color, bin_img_color, element);

#if SHOW_BIN_IMG == 1
    imshow("src_img", bin_img_color);
#endif
    //保存处理后的图片
    this->mask = bin_img_color;
    this->gray_img = bin_img_gray;
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
    this->img_cols = mask.cols;
    this->img_rows = mask.rows;
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
#endif
            }
        }
    }
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

            if (error_angle < 8.0f)
            {
                if (this->light_judge(light_left, light_right))
                {
                    if (this->average_color(this->armor_rect(light_left, light_right, src_img, error_angle)) < 50)
                    {
                        success_armor++;
                    }
                }
            }
        }
    }
    return success_armor;
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
    int left_h = MAX(light[i].size.height, light[i].size.width);
    int left_w = MIN(light[i].size.height, light[i].size.width);
    int right_h = MAX(light[j].size.height, light[j].size.width);
    int right_w = MIN(light[j].size.height, light[j].size.width);

    if ((left_h < right_h * 1.3 && left_w < right_w * 1.3) || (left_h > right_h * 0.7 && left_w > right_w * 0.7))
    {
        float h_max = (left_h + right_h) / 2.0f;
        // 两个灯条高度差不大
        if (fabs(light[i].center.y - light[j].center.y) < 0.8f * h_max)
        {
            //装甲板长宽比
            float w_max = light[j].center.x - light[i].center.x;
            if (w_max < h_max * 2.6f && w_max > h_max * 0.6)
            {
                return true;
            }
            if (w_max > h_max * 2.7f && w_max < h_max * 4.1f && fabs(light[i].angle - light[j].angle) < 10.0f)
            {
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
    RotatedRect rects = RotatedRect(
        Point((light[i].center.x + light[j].center.x) / 2, (light[i].center.y + light[j].center.y) / 2),
        Size(Distance(light[i].center, light[j].center) - (light[i].size.width + light[j].size.width), (light[i].size.height + light[j].size.height) / 2),
        angle);
    this->armor.push_back(rects); //储存装甲板旋转矩形
    Rect _rect = rects.boundingRect();
    Mat roi;
    if (_rect.y > 0 && _rect.y + _rect.height < 480)
    {
        roi = src_img(_rect);
    }
    return roi;
}

/**
 * @brief 多个装甲板筛选
 * 
 * @return Point 返回离图像中心点最近的装甲板中心点
 */
int LightBar::optimal_armor(bool judge)
{
    //图像中心点
    Point img_center(this->img_cols / 2, this->img_rows / 2);
    //筛选装甲板离中心点最小的位置
    int dist_min = 10000, num = 0;
    for (size_t i = 0; i < this->armor.size(); i++)
    {
        int dist = Distance(this->armor[i].center, img_center);
        if (dist < dist_min)
        {
            dist_min = dist;
            num = i;
        }
    }

#if ROI_IMG == 1
    if (judge) //图像ROI
    {
        int _x = roi_rect.boundingRect().x + this->armor[num].center.x;
        int _y = roi_rect.boundingRect().y + this->armor[num].center.y;
        RotatedRect _rect = RotatedRect(
            Point(_x, _y),
            Size(this->armor[num].size.width, this->armor[num].size.height),
            this->armor[num].angle);
        this->armor[num] = _rect; //储存更新完后装甲板位置
        // cout << "armor = " << _rect.center << _rect.size << endl;
        //更新下一帧ROI位置
        this->roi_rect = RotatedRect(
            this->armor[num].center,
            Size(this->armor[num].size.width * 8, this->armor[num].size.height * 8),
            this->armor[num].angle);
        // cout << "roi = " << roi_rect.center << roi_rect.size << endl;
        // cout << endl;
    }
    else
    {
        RotatedRect _rect = RotatedRect(
            this->armor[num].center,
            Size(IMG_COLS / 4, IMG_ROWS / 4),
            this->armor[num].angle);
        this->roi_rect = _rect; //储存下一帧ROI位置
    }
#endif
    return num;
}

/**
 * @brief 清除vector数据
 * 
 */
void LightBar::eliminate()
{
    light.clear();
    armor.clear();
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
            Point(IMG_COLS / 2, IMG_ROWS / 2),
            Size(IMG_COLS, IMG_ROWS),
            this->roi_rect.angle);
    }
}