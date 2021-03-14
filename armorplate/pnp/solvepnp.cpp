#include "solvepnp.h"

/**
 * @brief 装甲板二维点
 * 
 * @param light_left 左灯条
 * @param light_right 右灯条
 * @param _w 实际宽度
 * @param _h 实际高度
 * @return float 返回计算实际深度
 */
void SolveP4p::arrange_Point(RotatedRect left_light, RotatedRect right_light)
{
    this->target2d.clear();
    vector<Point2f>(target2d).swap(target2d);
    int left_w = MIN(left_light.size.height, left_light.size.width);
    int left_h = MAX(left_light.size.height, left_light.size.width);
    int right_w = MIN(right_light.size.height, right_light.size.width);
    int right_h = MAX(right_light.size.height, right_light.size.width);
    //左上
    this->target2d.push_back(Point(left_light.center.x - (left_w), left_light.center.y - (left_h/2)));
    this->target2d.push_back(Point(right_light.center.x + (right_w), right_light.center.y - (right_h/2)));
    this->target2d.push_back(Point(right_light.center.x + (right_w), right_light.center.y + (right_h/2)));
    this->target2d.push_back(Point(left_light.center.x - (left_w), left_light.center.y + (left_h/2)));
    // return this->run_SolvePnp(_w, _h);
}

/**
 * @brief Solvep4p
 * 
 * @param _W 实际宽度
 * @param _H 实际高度
 * @return float 返回深度
 */
void SolveP4p::run_SolvePnp(float _W, float _H)
{
    float half_x = _W * 0.5;
    float half_y = _H * 0.5;

    object_3d.clear();
    vector<Point3f>(object_3d).swap(object_3d);

    object_3d.push_back(Point3f(-half_x, -half_y, 0));
    object_3d.push_back(Point3f(half_x, -half_y, 0));
    object_3d.push_back(Point3f(half_x, half_y, 0));
    object_3d.push_back(Point3f(-half_x, half_y, 0));
    // cout<<target2d.size()<<endl;
    // cout<<object_3d.size()<<endl;
    solvePnP(object_3d, target2d, cameraMatrix, distCoeffs, rvec, tvec, false, SOLVEPNP_ITERATIVE);

    Mat ptz = camera_ptz(tvec); //云台Pitch轴当前角度
    get_Angle(ptz);
    rvec.release();
    tvec.release();
    ptz.release();
}
/**
 * @brief 计算深度信息
 * 
 * @param t 传入平移向量
 * @return Mat 返回世界坐标系
 */
Mat SolveP4p::camera_ptz(Mat &t)
{
    //设相机坐标系绕X轴你是逆时针旋转θ后与云台坐标系的各个轴向平行
    // double theta = 0; /*-atan(static_cast<double>(ptz_camera_y + barrel_ptz_offset_y))/static_cast<double>(overlap_dist);*/
    // double r_data[] = {1, 0, 0, 0, cos(theta), sin(theta), 0, -sin(theta), cos(theta)};
    //设相机坐标系的原点在云台坐标系中的坐标为(x0,y0,z0)
    double t_data[] = {static_cast<double>(ptz_camera_x), static_cast<double>(ptz_camera_y), static_cast<double>(ptz_camera_z)};
    // Mat r_camera_ptz(3, 3, CV_64FC1, r_data);
    Mat t_camera_ptz(3, 1, CV_64FC1, t_data);
    Mat position_in_ptz = /* r_camera_ptz * */ t - t_camera_ptz;
    //cout << position_in_ptz << endl;
    t_camera_ptz.release();
    return position_in_ptz;
}

void SolveP4p::run_SolvePnp_Buff(Mat &srcImg, float buff_angle, float _W, float _H)
{
    float half_x = _W * 0.5;
    float half_y = _H * 0.5;

    object_3d.clear();
    object_3d.push_back(Point3f(-half_x, -half_y, 0));
    object_3d.push_back(Point3f(half_x, -half_y, 0));
    object_3d.push_back(Point3f(half_x, half_y, 0));
    object_3d.push_back(Point3f(-half_x, half_y, 0));

    solvePnP(object_3d, target2d, cameraMatrix, distCoeffs, rvec, tvec, false, SOLVEPNP_ITERATIVE);

    // draw_Coordinate(srcImg);

    Mat ptz = camera_ptz(tvec); //云台Pitch轴当前角度
    //cout << ptz << "-----" << rect.center << endl;

    get_Angel_Buff(ptz, buff_angle); //输入云台Pitch轴当前角度,目标矩形位置
}

void SolveP4p::draw_Coordinate(Mat &input)
{
    vector<Point2f> reference_Img;
    vector<Point3f> reference_Obj;
    reference_Obj.clear();
    reference_Img.clear();
    vector<Point2f>(reference_Img).swap(reference_Img);
    vector<Point3f>(reference_Obj).swap(reference_Obj);
    reference_Obj.push_back(Point3f(0.0, 0.0, 0.0));
    reference_Obj.push_back(Point3f(100, 0.0, 0.0));
    reference_Obj.push_back(Point3f(0.0, 100, 0.0));
    reference_Obj.push_back(Point3f(0.0, 0.0, 100));

    projectPoints(reference_Obj, rvec, tvec, cameraMatrix, distCoeffs, reference_Img);

    line(input, reference_Img[0], reference_Img[1], Scalar(0, 0, 255), 2);
    line(input, reference_Img[0], reference_Img[2], Scalar(0, 255, 0), 2);
    line(input, reference_Img[0], reference_Img[3], Scalar(255, 0, 0), 2);

    reference_Img.clear();
    reference_Obj.clear();

    vector<Point2f>(reference_Img).swap(reference_Img);
    vector<Point3f>(reference_Obj).swap(reference_Obj);

#if SHOW_OUTPUT_IMG == 1
// imshow("outImg", input);
#endif
}

/*--------------------------------------大神符------------------------------------------*/
/**
 * @brief Get the Angel Buff object
 * 
 * @param pos_in_ptz persent angle of ptz pitch axis
 * @param buff_angle target's rect position
 */
void SolveP4p::get_Angel_Buff(const Mat &pos_in_ptz, float buff_angle)
{
    //计算子弹下坠补偿
    const double *_xyz = (const double *)pos_in_ptz.data;
    // cout << "x:" << _xyz[0] << "   y:" << _xyz[1] << "   z:" << _xyz[2] << endl;

    //    double down_t = 0.0;
    //    if(BULLET_SPEED > 10e-3)
    //        down_t = _xyz[2] / BULLET_SPEED;

    //    double offset_gravity = 0.5 * 9.8 * down_t*down_t;
    double z = _xyz[2];
    float target_h = 0;

    //        float buff_bottom_h = 519;
    //        float robot_h = 400;
    //        float buff_robot_z = 7200;
    float buff_robot_y = BUFF_BOTTOM_H - ROBOT_H; //大能量机关最低部装甲板到枪口高度
    // float predict_buff_angle = buff_angle + PRE_ANGLE;
    // if(predict_buff_angle > 360)
    //     predict_buff_angle = predict_buff_angle - 360.0f;

    float buff_h = 800 * sin(buff_angle * CV_PI / 180) + 800; // 计算风车相对最底面装甲高度　０－１６００
    target_h = buff_robot_y + buff_h;                         //目标装甲板与步兵枪口的高度
    float distance = sqrt(pow(target_h, 2) + pow(BUFF_ROBOT_Z, 2));
    z = distance;
    //cout << predict_buff_angle << endl;
    //cout << buff_h << endl;

    // double xyz[3] = {_xyz[0], _xyz[1] - OFFSET_Y_BARREL_PTZ, z};
    double xyz[3] = {_xyz[0], _xyz[1] - OFFSET_Y_BARREL_PTZ, _xyz[2]};

    //计算角度
    // double alpha = 0.0, theta = 0.0;
    //    alpha = asin(offset_y_barrel_ptz/sqrt(xyz[1]*xyz[1] + xyz[2]*xyz[2]));//offset_y_barrel_ptz并未使用
    //    cout<<"aplha: "<<alpha<<endl;
    // theta = atan(xyz[1]/xyz[2]);//后续还要进行测试比较角度的准确性和a
    /*------------------------------------北理珠---------------------------------------------------*/
    float thta = -static_cast<float>(atan2(xyz[1], xyz[2]));          // 云台与目标点的相对角度
    float balta = static_cast<float>(atan2(target_h, xyz[2])) - thta; // 云台与地面的相对角度
    /*------------------------------------北理珠---------------------------------------------------*/

    //该部分参考北理珠，由于balta角的效果还有待考证，这里就先不加入，只是调整了坐标系
    //    angle_y = -(theta-alpha);

    //cout << xyz[1] << endl;
    //    if(xyz[1] < 0){
    //        theta = atan(-xyz[1]/xyz[2]);
    //        angle_y = -(alpha+theta);  // camera coordinate
    //    }
    //    else if (xyz[1] < offset_y_barrel_ptz){
    //        theta = atan(xyz[1]/xyz[2]);
    //        angle_y = -(alpha-theta);  // camera coordinate
    //    }
    //    else{
    //        theta = atan(xyz[1]/xyz[2]);
    //        angle_y = (theta-alpha);   // camera coordinate
    //    }

    angle_y = -getBuffPitch(z / 1000, (target_h) / 1000, BULLET_SPEED);
    /*------------------------------------北理珠---------------------------------------------------*/
    angle_y += balta;
    /*------------------------------------北理珠---------------------------------------------------*/

    angle_x = static_cast<float>(atan2(xyz[0], xyz[2]));
    //test
    angle_y = static_cast<float>(atan2(xyz[1], xyz[2]));
    angle_x = angle_x * 180 / CV_PI;
    angle_y = angle_y * 180 / CV_PI;
    dist = xyz[2];

    //    cout << "angle_x:" << angle_x << "     angle_y:" << angle_y << "    dist:" << dist <<endl;
}

/**
 * @brief 获取Pitch轴角度
 * 
 * @param dist 距离
 * @param tvec_y 枪口与目标装甲板的垂直距离
 * @param ballet_speed 子弹速度
 * @return float 
 */
float SolveP4p::getBuffPitch(float dist, float tvec_y, float ballet_speed)
{
    // 申明临时y轴方向长度,子弹实际落点，实际落点与击打点三个变量不断更新（mm）
    float y_temp, y_actual, dy;
    // 重力补偿枪口抬升角度
    float a = 0.0;
    float GRAVITY = 9.7887f; //shenzhen 9.7887  zhuhai
    y_temp = tvec_y;
    // 迭代求抬升高度
    for (int i = 0; i < 10; i++)
    {
        // 计算枪口抬升角度
        a = (float)atan2(y_temp, dist);
        // 计算实际落点
        float t;
        t = dist / (ballet_speed * cos(a));
        y_actual = ballet_speed * sin(a) * t - GRAVITY * t * t / 2;
        dy = tvec_y - y_actual;
        y_temp = y_temp + dy;
        // 当枪口抬升角度与实际落点误差较小时退出
        if (fabsf(dy) < 0.01)
        {
            break;
        }
    }
    return a;
}

void SolveP4p::vertex_Sort(RotatedRect &box)
{
    Point2f vertex[4];
    Point2f lu, ld, ru, rd;

    box.points(vertex); //box的点存储到vertex中
    //对顶点点进行排序
    sort(vertex, vertex + 4, [](const Point2f &p1, const Point2f &p2) { return p1.x < p2.x; });

    if (vertex[0].y < vertex[1].y)
    {
        lu = vertex[0];
        ld = vertex[1];
    }
    else
    {
        lu = vertex[1];
        ld = vertex[0];
    }
    if (vertex[2].y < vertex[3].y)
    {
        ru = vertex[2];
        rd = vertex[3];
    }
    else
    {
        ru = vertex[3];
        rd = vertex[2];
    }

    target2d.clear();
    vector<Point2f>(target2d).swap(target2d); 
    target2d.push_back(lu);
    target2d.push_back(ru);
    target2d.push_back(rd);
    target2d.push_back(ld);
}

/**
 * @brief Get the Angle object
 * 
 * @param pos_in_ptz 
 */
void SolveP4p::get_Angle(const Mat &pos_in_ptz)
{
    //计算子弹下坠补偿
    const double *_xyz = (const double *)pos_in_ptz.data;
    //cout << "x:" << _xyz[0] << "   y:" << _xyz[1] << "   z:" << _xyz[2] << endl;
    double bullet_speed = 1.0;
    double down_t = 0.0;
    if (bullet_speed > 10e-3)
    {
        down_t = _xyz[2] / bullet_speed;
    }

    // double offset_gravity = 0.5 * 9.8 * down_t * down_t;
    double xyz[3] = {_xyz[0], _xyz[1] /*- offset_gravity*/, _xyz[2]};

    //计算角度
    if (barrel_ptz_offset_y != 0.f)
    {
        double alpha = 0.0, Beta = 0.0;
        alpha = asin(static_cast<double>(barrel_ptz_offset_y) / sqrt(xyz[1] * xyz[1] + xyz[2] * xyz[2]));

        if (xyz[1] < 0)
        {
            Beta = atan(-xyz[1] / xyz[2]);
            angle_y = static_cast<float>(-(alpha + Beta)); //camera coordinate
        }
        else if (xyz[1] < static_cast<double>(barrel_ptz_offset_y))
        {
            Beta = atan(xyz[1] / xyz[2]);
            angle_y = static_cast<float>(-(alpha - Beta));
        }
        else
        {
            Beta = atan(xyz[1] / xyz[2]);
            angle_y = static_cast<float>((Beta - alpha)); // camera coordinate
        }
    }
    else
    {
        angle_y = static_cast<float>(atan2(xyz[1], xyz[2]));
    }
    if (barrel_ptz_offset_x != 0.f)
    {
        double alpha = 0.0, Beta = 0.0;
        alpha = asin(static_cast<double>(barrel_ptz_offset_x) / sqrt(xyz[0] * xyz[0] + xyz[2] * xyz[2]));
        if (xyz[0] > 0)
        {
            Beta = atan(-xyz[0] / xyz[2]);
            angle_x = static_cast<float>(-(alpha + Beta)); //camera coordinate
        }
        else if (xyz[0] < static_cast<double>(barrel_ptz_offset_x))
        {
            Beta = atan(xyz[0] / xyz[2]);
            angle_x = static_cast<float>(-(alpha - Beta));
        }
        else
        {
            Beta = atan(xyz[0] / xyz[2]);
            angle_x = static_cast<float>(Beta - alpha); // camera coordinate
        }
    }
    else
    {
        angle_x = static_cast<float>(atan2(xyz[0], xyz[2]));
    }
    angle_x = static_cast<float>(angle_x) * 180 / CV_PI;
    angle_y = static_cast<float>(angle_y) * 180 / CV_PI;
    dist = static_cast<float>(xyz[2]);
#if SHOW_ANGLE_INFORMATION == 1
    cout << "angle_x:" << angle_x << "     angle_y:" << angle_y << "    dist:" << dist << endl;
#endif
}