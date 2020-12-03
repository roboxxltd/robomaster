#include "armorplate.h"

armorplate::armorplate() {}

armorplate::~armorplate() {}

void armorplate::Pretreatment()
{
    VideoCapture capture("/home/xx/下载/效果图/armor_4.avi");
    if (!capture.isOpened())
    {
        cout << "无法打开相机..." << endl;
    }
    /*原图像*/
    namedWindow("frame", WINDOW_AUTOSIZE);
    /*处理后图像*/
    namedWindow("mask", WINDOW_AUTOSIZE);
    /*滑动条窗口*/
    namedWindow("Control", WINDOW_AUTOSIZE);
    namedWindow("draw", WINDOW_AUTOSIZE);
    Mat frame;
    Mat result_img;
    int input = 0;
    // int gray_thre_min = 60;
    int gray_thre_min = 8;
    int thre_min = 40;
    int ddd = 20;
    Mat gray_img;
    Mat src_img;
    Mat gray_color_img;
    /*二值化图片*/
    Mat mask;
    /*开运算参数*/
    Mat kernel_open = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1, -1));
    /*闭运算操作*/
    Mat kernel_close = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1, -1));
    /*膨胀参数*/
    Mat kernel_dilate = getStructuringElement(MORPH_RECT, Size(3, 7), Point(-1, -1));
    /*腐蚀参数*/
    Mat kernel_erode = getStructuringElement(MORPH_RECT, Size(3, 1), Point(-1, -1));
    int light_left = 0;  //左灯条下标
    int light_right = 0; //右灯条下标
    int lost = 0;        //丢失次数
    for (;;)
    {
        double t = (double)cv::getTickCount(); //开始计时
        capture >> frame;                      //读取当前帧
        Center = Point(0, 0);
        resize(frame, src_img, Size(640, 480));
        if (success_armor)
        {
            src_img = src_img(armor_roi);
            success_armor = false;
        }
        //创建进度条
#if SLIDE_BAR_ON == 1
        createTrackbar("gra", "Control", &ddd, 100);
        createTrackbar("gra", "gray", &gray_thre_min, 255);
        createTrackbar("thre", "Control", &thre_min, 255);
#endif
        input = 0;
        this->draw_img = Mat::zeros(Size(640, 480), CV_8UC3);
        //转灰度图
        cvtColor(src_img, gray_img, COLOR_BGR2GRAY);
        vector<Mat> channels;
        split(src_img, channels);
        Mat dst_img;
        bitwise_and(channels.at(2), (channels.at(2) - channels.at(0)) + (channels.at(2) - channels.at(1)), dst_img);

        threshold(dst_img, mask, thre_min, 255, THRESH_BINARY);

        // channels.at(0) = channels.at(0) - channels.at(2) + channels.at(1);
        // threshold(channels.at(0), mask, thre_min, 255, THRESH_BINARY);

        threshold(gray_img, gray_color_img, gray_thre_min, 255, THRESH_BINARY);
        // 形态学操作
        morphologyEx(mask, mask, MORPH_CLOSE, kernel_close, Point(-1, -1)); //闭操作
        // morphologyEx(mask, mask, MORPH_OPEN, kernel_open, Point(-1, -1));   //开操作
        // erode(mask, mask, kernel_erode, Point(-1, -1));
        dilate(mask, mask, kernel_dilate, Point(-1, -1)); //膨胀
        imshow("Control", mask);
        // 轮廓增强
        Canny(mask, mask, 3, 9, 3);
        /*所有轮廓旋转矩形*/
        RotatedRect minRect;
        /*轮廓周长*/
        int perimeter = 0;

        findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
        //筛选，去除一部分矩形
        for (size_t i = 0; i < contours.size(); ++i)
        {
            perimeter = arcLength(contours[i], true); //轮廓周长
            if (perimeter > 50 && perimeter < 4000 && contours[i].size() >= 5)
            {
                //椭圆拟合
                minRect = fitEllipse(Mat(contours[i]));
                //重新定义长宽和角度
                if (minRect.angle > 90.0f)
                    minRect.angle = minRect.angle - 180.0f;
                //灯条长宽比
                float light_w_h;
                if (minRect.size.height != 0)
                    light_w_h = minRect.size.width / minRect.size.height;
                else
                    continue;
                //判断长宽和角度
                if (fabs(minRect.angle) < 30 && light_w_h < 0.6f)
                {
                    light.push_back(minRect);
                    // 绘制灯条
#ifdef DRAWIMG == 1
                    // Point2f vertex[4];
                    // minRect.points(vertex);
                    // for (int l = 0; l < 4; l++)
                    // {
                    //     line(this->draw_img, vertex[l], vertex[(l + 1) % 4], Scalar(0, 255, 255), 3, 8);
                    //     line(frame, vertex[l], vertex[(l + 1) % 4], Scalar(0, 255, 255), 3, 8);
                    // }
#endif
                }
            }
        }
        /*装甲板中心点*/
        //遍历所有矩形，两两组合
        int nnn = 0;
        for (size_t i = 0; i < light.size(); ++i)
        {
            for (size_t j = i + 1; j < light.size(); ++j)
            {
                float error_angle;
                if (abs(light[i].center.x - light[j].center.x) < light[i].size.height)
                    continue;
                else
                    error_angle = atan((light[i].center.y - light[j].center.y) / (light[i].center.x - light[j].center.x));
                if (error_angle < 8.0f) //两个灯条不超过这个误差角度
                {
                    //两个灯条比较判断
                    if (this->Light_judge(i, j))
                    {

                        if (light[i].center.x >= light[j].center.x)
                        {
                            light_left = j;
                            light_right = i;
                        }
                        else
                        {
                            light_left = i;
                            light_right = j;
                        }
                        //储存装甲板Rect位置
                        Rect rects = Rect(int(light[light_left].center.x),
                                          int((light[light_right].center.y + light[light_left].center.y) / 2 - (light[light_right].size.height + light[light_left].size.height) / 4),
                                          int(light[light_right].center.x - light[light_left].center.x),
                                          int((light[light_right].size.height + light[light_left].size.height) / 2));
                        Mat roi = gray_img(rects);
                        //                        cout << this->Average_color(roi) << endl;
                        if (this->Average_color(roi) < 50)
                        {
                            // rectangle(draw_img, rects, Scalar(0, 255, 0), 9, 8);
                            // rectangle(frame, rects, Scalar(0, 0, 255), 3, 8);
                            armor_rects = rects; //储存装甲板Rect 方便数字识别
                            success_armor = true;
                            left.push_back(light_left);
                            right.push_back(light_right);
                            lost = 0;
                            nnn++;
                        }
                    }
                }
            }
        }
        //丢失帧数++
        if (!success_armor)
        {
            lost++;
            //丢失目标扩大寻找范围
            int x = armor_roi.x - (armor_roi.width * 3 / 2);
            int y = armor_roi.y - (armor_roi.height / 2);
            int w = armor_roi.width * 4;
            int h = armor_roi.height * 2;
            if (lost == 1)
            {
                if (x > 0 && w + x < 640)
                {
                    if (h + y < 480 && y > 0)
                    {
                        armor_roi = Rect(x, y, w, h);
                    }
                    else
                    {
                        armor_roi = Rect(x, 0, w, 480);
                    }
                }
                else
                {
                    if (h + y < 480 && y > 0)
                    {
                        armor_roi = Rect(0, y, 640, h);
                    }
                    else
                    {
                        armor_roi = Rect(0, 0, 640, 480);
                    }
                }
                success_armor = true;
            }
        }
        else
        {
            Strike_max();
            cout << nnn << endl;
        }
        input++; //图片数量
                 //绘画相对击打位置
#ifdef DRAWIMG == 1
        circle(draw_img, this->Center, 5, Scalar(0, 255, 0), -1);
        circle(frame, this->Center, 5, Scalar(0, 255, 0), -1);
        // rectangle(draw_img, armor_rects, Scalar(0, 0, 255), 3, 8);
        rectangle(frame, armor_roi, Scalar(0, 0, 255), 1, 8);
#endif
        //清除数据
        Eliminate();
        imshow("draw", this->draw_img);
        imshow("frame", frame);
        imshow("mask", mask);

        imshow("gray", gray_color_img);
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency(); //结束计时
        int fps = int(1.0 / t);                                        //转换为帧率
        // cout << "FPS: " << fps << endl;                                //输出帧率
        char c = waitKey(0);
        if (c == 27) //"Esc"-
        {
            break;
        }
    }
    capture.release(); //释放视频内存
}

/**
 * @brief 
 * 
 * @param i 
 * @param j 
 * @return true 
 * @return false 
 */
bool armorplate::Light_judge(int i, int j)
{
    if (light[i].size.height * 0.7f < light[j].size.height && light[i].size.width * 1.3f > light[j].size.width)
    {
        float armor_width = fabs(light[i].center.x - light[j].center.x);
        if (armor_width > light[i].size.width * 0.8 && armor_width < (light[i].size.height + light[j].size.height) * 2)
        {
            float h_max = (light[i].size.height + light[j].size.height) / 2.0f;
            // 两个灯条高度差不大
            if (fabs(light[i].center.y - light[j].center.y) < 0.5f * h_max)
            {
                //装甲板长宽比
                float w_max = Distance(light[i].center, light[j].center);
                // cout << w_max << " " << h_max << endl;
                if (w_max < h_max * 2.0f && w_max > h_max)
                    return true;
                if (w_max > h_max * 3.0f && w_max < h_max * 3.5f)
                    return true;
            }
        }
    }
    return false;
}

int armorplate::Average_color(Mat roi)
{
    int average_intensity = static_cast<int>(mean(roi).val[0]);
    return average_intensity;
}

void armorplate::Strike_max()
{
    int num = 0;
    Point armor_point, img_point;
    img_point = Point(draw_img.rows / 2, draw_img.cols / 2); //图像中点

    for (size_t i = 0; i < left.size(); i++)
    {
        armor_level.clear();
        //判断灯条之间的高度差
        float light_h = MIN(light[left[i]].size.height, light[right[i]].size.height); //最小灯条

        if (fabs(light[left[i]].center.y - light[right[i]].center.y) < light_h / 2) //灯条中心点的差距不超过最小灯条的一半
        {
            armor_level.push_back(true);
            cout << "y " << fabs(light[left[i]].center.y - light[right[i]].center.y) << " " << light_h << endl;
        }
        else
        {
            armor_level.push_back(false);
        }

        //判断两个灯条之间的角度
        if (fabs(light[left[i]].angle - light[right[i]].angle) < 45.0f) //角度和小于30
        {
            armor_level.push_back(true);
            cout << "angle " << fabs(light[left[i]].angle - light[right[i]].angle) << endl;
        }
        else
        {
            armor_level.push_back(false);
        }

        if (fabs(light[left[i]].size.height - light[right[i]].size.height) < 20) //小于15个像素点
        {
            armor_level.push_back(true);
            cout << "height " << fabs(light[left[i]].size.height - light[right[i]].size.height) << endl;
        }
        else
        {
            armor_level.push_back(false);
        }

        // //判断大小装甲板, 优先击打小装甲板
        cout << light[left[i]].angle - light[right[i]].angle << endl;
        if ((light[left[i]].angle - light[right[i]].angle) > 9.0f) //假大装甲板
        {
            armor_level.push_back(false);
        }
        else
        {
            armor_level.push_back(true);
        }

        if (this->Armor_judge())
        {
            num = i;
        }
    }

    speed_x = (light[right[num]].center.x + light[left[num]].center.x) / 2 - Center.x; //记录两次中心点x轴的像素点差
    speed_y = (light[right[num]].center.y + light[left[num]].center.y) / 2 - Center.y; //记录两次中心点y轴的像素点差

    Center = Point((light[right[num]].center.x + light[left[num]].center.x) / 2 + armor_roi.x,
                   (light[right[num]].center.y + light[left[num]].center.y) / 2 + armor_roi.y);

    armor_rects = Rect(
        int(light[left[num]].center.x + armor_roi.x),
        int((light[right[num]].center.y + light[left[num]].center.y) / 2 - (light[right[num]].size.height + light[left[num]].size.height) / 4 + armor_roi.y),
        int(light[right[num]].center.x - light[left[num]].center.x),
        int((light[right[num]].size.height + light[left[num]].size.height) / 2));

    // cout << "rect" << light[left[num]].center.x << " " << light[left[num]].center.y << endl;
    int x = abs(armor_rects.x - armor_rects.width * 3 / 4);
    int y = abs(armor_rects.y - (armor_rects.height * 3 / 4));
    int w = abs(armor_rects.width * 5 / 2);
    int h = abs(armor_rects.height * 5 / 2);
    if (y + h > 480 || x + w > 640)
    {
        armor_roi = Rect(0, 0, 640, 480);
    }
    else
    {
        armor_roi = Rect(x, y, w, h);
    }
    cout << endl;

    // cout << armor_roi.x << " " << armor_roi.y << " " << armor_roi.width << " " << armor_roi.height << endl;
}

void armorplate::Eliminate()
{
    light.clear();
    contours.clear();
    left.clear();
    right.clear();
}

bool armorplate::Armor_judge()
{
    int num = 0;
    for (size_t i = 0; i < armor_level.size(); i++)
    {
        if (armor_level[i] == true)
        {
            num++;
        }
    }
    if (num == 4)
    {
        return true;
    }
    cout << "level " << num << endl;
    return false;
}