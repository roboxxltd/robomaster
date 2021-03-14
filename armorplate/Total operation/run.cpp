#include "run.h"

WorKing::WorKing()
    : capture(USB_CAPTURE_DEFULT), cap(ISOPEN_INDUSTRY_CAPTURE) {}

/**
 * @brief 运行函数
 *
 */
void WorKing::Run()
{
  int offset_x = 0;
  int offset_y = 500;
  int _offset_x = 1;
  int _offset_y = 0;
#if IS_PARAM_ADJUSTMENT == 1
  namedWindow("parameter", WINDOW_AUTOSIZE);

  createTrackbar("offset_x:", "parameter", &offset_x, 2000, NULL);
  createTrackbar("offset_y:", "parameter", &offset_y, 2000, NULL);
  createTrackbar("_offset_x:", "parameter", &_offset_x, 1, NULL);
  createTrackbar("_offset_y:", "parameter", &_offset_y, 1, NULL);
// #if ENEMY_COLOR == 1
//   createTrackbar("GRAY_TH_RED:", "parameter", &img.red_armor_gray_th, 255);
//   createTrackbar("COLOR_TH_RED:", "parameter", &img.red_armor_color_th, 255);
// #endif
// #if ENEMY_COLOR == 0
//   createTrackbar("GRAY_TH_BLUE:", "parameter", &img.blue_armor_gray_th, 255, NULL);
//   createTrackbar("COLOR_TH_BLUE:", "parameter", &img.blue_armor_color_th, 255, NULL);
// #endif
#endif
  for (;;)
  {
#if FPS_SHOW == 1
    double t = (double)cv::getTickCount(); //开始计时
#endif                                     // #endif
    if (cap.isindustryimgInput())
    {
      frame = cvarrToMat(cap.iplImage, true);
    }
    else
    {
      capture >> frame;
    }
    int ctrl_arr[REC_BUFF_LENGTH];
    serial.RMreceiveData(ctrl_arr);
    int enemy_color = 1;
    enemy_color = ctrl_arr[1];
    // armor.success_armor = false;
    Mat src_img;
    src_img = frame;
#if ROI_IMG == 1
    // ROI
    if (armor.lost_success_armor)
    {
      src_img = frame(armor.armor_roi);
      // cout<<armor.armor_roi<<endl;
    }
    else
    {
      src_img = frame; //直接赋值
      armor.armor_roi = Rect(0, 0, 0, 0);
    }
#endif
    
    //图像预处理
    img.pretreat(src_img, enemy_color);

    //找到灯条后
    if (rgb.find_light(img.mask))
    {
      //装甲板大于等于1块时
      if (rgb.armor_fitting(img.gray_img))
      {

        armor.rect_num = rgb.optimal_armor() / 2;
#if CALL_DEPTH_INFORMATION == 1

        // float depth = 0;
        int box_x = rgb.armor[armor.rect_num].center.x + armor.armor_roi.x;
        int box_y = rgb.armor[armor.rect_num].center.y + armor.armor_roi.y;
        
        // if(box_x >= CAMERA_RESOLUTION_COLS)
        // {
        //   box_x = CAMERA_RESOLUTION_COLS;

        // }
        // if(box_y >=CAMERA_RESOLUTION_ROWS)
        // {
        //   box_y = CAMERA_RESOLUTION_ROWS;
        // }
        RotatedRect box = RotatedRect(Point(box_x, box_y), rgb.armor[armor.rect_num].size,
                        rgb.armor[armor.rect_num].angle);
        RotatedRect left_light = RotatedRect(Point(rgb.light[rgb.light_subscript[armor.rect_num]].center.x+armor.armor_roi.x, rgb.light[rgb.light_subscript[armor.rect_num]].center.y+armor.armor_roi.y),
                                  rgb.light[rgb.light_subscript[armor.rect_num]].size, rgb.light[rgb.light_subscript[armor.rect_num]].angle);
        RotatedRect right_light = RotatedRect(Point(rgb.light[rgb.light_subscript[armor.rect_num+1]].center.x+armor.armor_roi.x, rgb.light[rgb.light_subscript[armor.rect_num+1]].center.y+armor.armor_roi.y),
                                  rgb.light[rgb.light_subscript[armor.rect_num+1]].size, rgb.light[rgb.light_subscript[armor.rect_num+1]].angle);
        // line(frame)
        rectangle(frame, box.boundingRect(), Scalar(0, 255, 0), 3, 8);
        rectangle(frame, left_light.boundingRect(), Scalar(0, 255, 255), 9, 8);
        rectangle(frame, right_light.boundingRect(), Scalar(0, 255, 255), 9, 8);
        pnp.vertex_Sort(box);
        // pnp.arrange_Point(left_light, right_light);
        float _w = MIN(box.size.width, box.size.height);
        float _h = MAX(box.size.width, box.size.height);
        // cout << _w / _h << endl;

        if (_w / _h >= 2.2)
        {
          pnp.run_SolvePnp(BIG_ARMORPLATE_WIDITH, ARMORPLATE_HIGHT);
        }
        else
        {
          pnp.run_SolvePnp(SMALL_ARMORPLATE_WIDTH, ARMORPLATE_HIGHT);
          
        }
        armor.yaw = pnp.angle_x;
        armor.pitch = pnp.angle_y;

        // cout<<armor.yaw<<endl;
        // cout<<armor.pitch<<endl;
        // //test 半径补偿

        if (armor.yaw < 0)
        {
          armor.yaw += armor.offset_ratio / 10;
        }
        else
        {
          armor.yaw -= armor.offset_ratio / 10;
        }

        if (armor.pitch > 0)
        {
          armor.pitch += armor.offset_ratio / 10;
        }
        else
        {
          armor.pitch -= armor.offset_ratio / 10;
        }
        // // test 半径补偿

        armor.depth = int(pnp.dist);
        // cout<<armor.depth<<endl;
        // armor.yaw = armor.yaw + offset_x / 100;
        // if(armor.depth <= 2000)
        // {
        //   armor.pitch = armor.pitch - (200/100);
        // }
        // else if(armor.depth > 2000 && armor.depth <= 2500)
        // {
        //     armor.pitch = armor.pitch - (450/100);
        // }
        // else if(armor.depth > 2500 && armor.depth <= 3000)
        // {
        //     armor.pitch = armor.pitch - (500/100);
        // }
        // else if(armor.depth > 3000 && armor.depth <= 3500)
        // {
        //     armor.pitch = armor.pitch - (592/100);
        // }
        // else if(armor.depth > 3500 && armor.depth <= 4000)
        // {
        //     armor.pitch = armor.pitch - (650/100);
        // }
        // else if(armor.depth > 4000 && armor.depth <= 4500)
        // {
        //     armor.pitch = armor.pitch - (700/100);
        // }
        // else if(armor.depth > 4500 && armor.depth <= 5000) 
        // {
        //   armor.pitch = armor.pitch - (750/100);
        // }
        // else
        // {
        //   armor.pitch = armor.pitch - (800/100);
        // }
        if (_offset_x == 0)
        {
          armor.yaw = armor.yaw - offset_x / 100;
        }
        else
        {
          armor.yaw = armor.yaw + offset_x / 100;
        }

        if (_offset_y == 0)
        {
          armor.pitch = armor.pitch - offset_y / 100;
        }
        else
        {
          armor.pitch = armor.pitch + offset_y / 100;
        }
        if (armor.yaw > 0)
        {
          armor._yaw = 0;
        }
        else
        {
          armor._yaw = 1;
        }
        if (armor.pitch > 0)
        {
          armor._pitch = 0;
        }
        else
        {
          armor._pitch = 1;
        }
        // cout << armor.yaw << " ," << armor.pitch << endl;


#endif

#if ROI_IMG == 1
        // ROI
        armor.success_armor = true; //识别正确

        if (armor.lost_success_armor)
        {
          int point_x = rgb.armor[armor.rect_num].center.x -
                        240 +
                        armor.armor_roi.x;
          int point_y = rgb.armor[armor.rect_num].center.y -
                        150 +
                        armor.armor_roi.y;
          int width = 480;
          int height = 300;
          if( width < CAMERA_RESOLUTION_COLS && height <CAMERA_RESOLUTION_ROWS)
          {
            if (point_x < 0)
            {
              point_x = 0;
            }
            if (point_y < 0)
            {
              point_y = 0;
            }
            if (point_x + width >= CAMERA_RESOLUTION_COLS)
            {
              width = CAMERA_RESOLUTION_COLS - abs(point_x);
            }
            if (point_y + height >= CAMERA_RESOLUTION_ROWS)
            {
              height = CAMERA_RESOLUTION_ROWS - abs(point_y);
            }
            armor.armor_roi = Rect(point_x, point_y, width, height);
            rgb.armor[armor.rect_num] = RotatedRect(
                Point(rgb.armor[armor.rect_num].center.x + armor.armor_roi.x,
                      rgb.armor[armor.rect_num].center.y + (armor.armor_roi.y)),
                rgb.armor[armor.rect_num].size, rgb.armor[armor.rect_num].angle);
            rectangle(frame, armor.armor_roi, Scalar(0, 255, 255), 3, 8);
          }
          
        }
        else
        {
          int point_x = rgb.armor[armor.rect_num].center.x - 240;
          int point_y = rgb.armor[armor.rect_num].center.y - 150;
          int width = 480;
          int height = 300;
          if (point_x < 0)
          {
            point_x = 0;
          }
          if (point_y < 0)
          {
            point_y = 0;
          }
          if (point_x + width >= CAMERA_RESOLUTION_COLS)
          {
            width = CAMERA_RESOLUTION_COLS - point_x;
          }
          if (point_y + height >= CAMERA_RESOLUTION_ROWS)
          {
            height = CAMERA_RESOLUTION_ROWS - point_y;
          }

          armor.armor_roi = Rect(point_x, point_y, width, height);
        }

#endif
        armor.lost_success_armor = armor.success_armor; //保存上一帧的参数
        // armor.yaw = rgb.armor[armor.rect_num].center.x;
        // armor.pitch = rgb.armor[armor.rect_num].center.y;
        // armor.is_shooting = 1;
        // armor.data_type = 1;
        // if(armor.yaw >= 640)
        // {
        //     armor._yaw = 0;
        // }
        // else{
        //     armor._yaw = 1;
        // }
        // if(armor.pitch >= 400)
        // {
        //     armor._pitch = 1;armor
        // }
        // else{
        //     armor._pitch = 0;
        // }
        // //绘图
        // rectangle(src_img, rgb.armor[armor.rect_num].boundingRect(),
        // Scalar(0, 255, 0), 3, 8); rectangle(src_img, armor.armor_roi,
        // Scalar(255, 200, 0), 3, 8);
        rgb.lost_armor = rgb.armor[armor.rect_num];
      }
      else //丢失目标
      {
        armor.success_armor = false;
        armor.lost_success_armor = armor.success_armor;
        armor.armor_roi = Rect(0, 0, 0, 0);
        armor.yaw = 0;
        armor.pitch = 0;
        armor._yaw = 0;
        armor._pitch = 0;
        armor.is_shooting = 0;
        armor.data_type = 0;
      }
    }
    else
    {
      armor.is_shooting = 0;
      armor.data_type = 0;
      armor.success_armor = false;
      armor.armor_roi = Rect(0, 0, 0, 0);
      armor.lost_success_armor = armor.success_armor;
      armor.yaw = 0;
      armor.pitch = 0;
      armor._yaw = 0;
      armor._pitch = 0;
    }
#if CALL_SERIALPORT == 1
    serial.RMserialWrite(armor._yaw, fabs(armor.yaw) * 100, armor._pitch,
                         fabs(armor.pitch) * 100, armor.depth, armor.data_type,
                         armor.is_shooting);
#endif
    imshow("frame", src_img);
    line(frame, Point(0, frame.rows / 2), Point(frame.cols, frame.rows / 2), Scalar(100, 100, 100), 3, 8);
    line(frame, Point(frame.cols / 2, 0), Point(frame.cols / 2, frame.rows), Scalar(100, 100, 100), 3, 8);
    imshow("111", frame);    
    
    src_img.release();
    frame.release();
    // serial.RMserialWrite();
    rgb.eliminate();
    armor.eliminate();
    // pnp.target2d.clear();
    // vector<Point2f> (pnp.target2d).swap(pnp.target2d);
    // pnp.object_3d.clear();
    // vector<Point3f> (pnp.object_3d).swap(pnp.object_3d);
    cap.cameraReleasebuff();
#if FPS_SHOW == 1
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency(); //结束计时
    int fps = int(1.0 / t);                                        //转换为帧率
    cout << "FPS: " << fps << endl;                                //输出帧率
#endif
    char c = waitKey(1);
    if (c == 27) //"Esc"-退出
    {
      break;
    }
  }
}

WorKing::~WorKing() {}

void WorKing::ddd()
{
  int n = 0;
  for (;;)
  {
    buff.yaw_data = 640;
    buff.pitch_data = 400;
    if (cap.isindustryimgInput())
    {
      frame = cvarrToMat(cap.iplImage, true);
    }
    imshow("frame", frame);
    char c = waitKey(1);
    if (c == 27) //"Esc"-退出
    {
      break;
    }
    else if (c == 'q' || c == 'k')
    {
      int num = 0;
      while (true)
      {
        char *cstr = new char[120];
        sprintf(cstr, "%s%d%s", "/home/sms/rm步兵蓝车数据集/", n, ".jpg");
        imwrite(cstr, frame);
        n++;
        num++;
        if (num >= 10)
        {
          break;
        }
      }
    }
    cap.cameraReleasebuff();
    cout << "第" << n << "张" << endl;
    // waitKey(2000);
  }
}

void WorKing::Run_MAX_Talisman()
{
  for (;;)
  {
#if FPS_SHOW == 1
    double t = (double)cv::getTickCount(); //开始计时
#endif                                     // #endif

    if (cap.isindustryimgInput())
    {
      frame = cvarrToMat(cap.iplImage, true);
    }
    else
    {
      capture >> frame;
    }

    Mat src_img;
    resize(frame, src_img,
           Size(CAMERA_RESOLUTION_COLS, CAMERA_RESOLUTION_ROWS));
    // if (true)
    // {
    // src_img = src_img(Rect(240, 0, 1000, 800));
    // }
    // else
    // {
    //     src_img = src_img; //直接赋值
    // }
    buff.pretreat(src_img, ENEMY_COLOR);
    if (buff.Looking_for_center())
    {
      int num = buff.Looking_for_target();
      float pre_center_angle;
      if (num >= 0)
      {
        if (buff.input != 0)
        {
          int _w = MAX(buff.max_buff_rects[buff.hit_subscript].size.width,
                       buff.max_buff_rects[buff.hit_subscript].size.height);
          int _h = MIN(buff.max_buff_rects[buff.hit_subscript].size.width,
                       buff.max_buff_rects[buff.hit_subscript].size.height);
          buff.Calculating_coordinates(buff.hit_subscript);
          pre_center_angle = atan2(buff.central_point.y - buff.pre_center.y,
                                   buff.central_point.x - buff.pre_center.x) *
                                 180 / CV_PI +
                             90;
          buff.rects_2d =
              RotatedRect(buff.pre_center, Size(_w, _h), pre_center_angle);
          Point2f vertex[4];
          buff.rects_2d.points(vertex);
          for (int i = 0; i < 4; i++)
          {
            line(src_img, vertex[i], vertex[(i + 1) % 4], Scalar(255, 100, 200),
                 5, CV_AA);
          }
        }
        else
        {
          buff.pre_center = buff.central_point;
          buff.rects_2d = RotatedRect(buff.pre_center,
                                      Size(MAX_BUFF_WIDTH, MAX_BUFF_HEIGHT), 0);
          rectangle(buff.src_img, buff.rects_2d.boundingRect(),
                    Scalar(0, 255, 0), 3, 8);
          pre_center_angle = 0;
        }
      }

      pnp.vertex_Sort(buff.rects_2d);
      pnp.run_SolvePnp_Buff(src_img, pre_center_angle, MAX_BUFF_WIDTH,
                            MAX_BUFF_HEIGHT);
      buff.yaw_data = pnp.angle_x;
      buff.pitch_data = pnp.angle_y;
      // //test 半径补偿
      if (buff.yaw_data < 0)
      {
        buff.yaw_data += buff.offset_ratio / 10;
      }
      else
      {
        buff.yaw_data -= buff.offset_ratio / 10;
      }

      if (buff.pitch_data > 0)
      {
        buff.pitch_data += buff.offset_ratio / 10;
      }
      else
      {
        buff.pitch_data -= buff.offset_ratio / 10;
      }
      // test 半径补偿

      buff.depth = int(pnp.dist);

      if (buff._offset_x == 0)
      {
        buff.yaw_data = buff.yaw_data - buff.offset_x / 100;
      }
      else
      {
        buff.yaw_data = buff.yaw_data + buff.offset_x / 100;
      }

      if (buff._offset_y == 0)
      {
        buff.pitch_data = buff.pitch_data - buff.offset_y / 100;
      }
      else
      {
        buff.pitch_data = buff.pitch_data + buff.offset_y / 100;
      }
      if (buff.yaw_data >= 0)
      {
        buff._yaw_data = 1;
      }
      else
      {
        buff._yaw_data = 0;
      }
      if (buff.pitch_data >= 0)
      {
        buff._pitch_data = 0;
      }
      else
      {
        buff._pitch_data = 1;
      }
    }
    else
    {
      buff.yaw_data = 6.4;
      buff.pitch_data = 4;
      buff._yaw_data = 0;
      buff._pitch_data = 0;
    }
    line(buff.src_img, Point(src_img.cols / 2, 0),
         Point(src_img.cols / 2, src_img.rows), Scalar(0, 255, 255), 3, 5);
    line(buff.src_img, Point(0, src_img.rows / 2),
         Point(src_img.cols, src_img.rows / 2), Scalar(0, 255, 255), 3, 5);
    imshow("frame", buff.src_img);
#if CALL_SERIALPORT == 1
    serial.RMserialWrite(buff._yaw_data, fabs(buff.yaw_data) * 100,
                         buff._pitch_data, fabs(buff.pitch_data) * 100,
                         buff.depth, 1, buff.diff_angle_);
    // cout<<fabs(buff.yaw_data) * 100<<endl;
#endif
#if FPS_SHOW == 1
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency(); //结束计时
    int fps = int(1.0 / t);                                        //转换为帧率
    cout << "FPS: " << fps << endl;                                //输出帧率
#endif

    buff.choice_success = false;
    buff.central_success = false;
    buff.armor_center.clear();
    buff.max_buff_rects.clear();
    buff.contours.clear();
    buff.hierarchy.clear();
    buff.buff.clear();

    cap.cameraReleasebuff();
    char c = waitKey(1);
    if (c == 27) //"Esc"-退出
    {
      break;
    }
  }
}