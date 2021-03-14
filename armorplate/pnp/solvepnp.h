#include "configure.h"
#include "control.h"
class SolveP4p
{
public:
    SolveP4p()
    {
        //读取摄像头标定xml文件
        FileStorage fs(CAMERA_PARAM_FILE, FileStorage::READ);
        //读取相机内参和畸变矩阵
        fs["camera-matrix"] >> cameraMatrix;
        fs["distortion"] >> distCoeffs;
        cout << cameraMatrix << endl;
        cout << distCoeffs << endl;
        cout << "RM_SolveAngle is readied" << endl;
    }
    ~SolveP4p() {}

    //坐标赋值
    void arrange_Point(RotatedRect left_light, RotatedRect right_light);
    // float armor_Point(RotatedRect left_light, float _w, float _h);
    void get_Angle(const Mat & pos_in_ptz);
    void run_SolvePnp(float _W, float _H);
    // float get_angle();
    // float calcu_depth();
    void vertex_Sort(RotatedRect &box);
    //大神符
    void run_SolvePnp_Buff(Mat &srcImg, float buff_angle, float _W, float _H);
    void draw_Coordinate(Mat &input);
    // void get_Angel_Buff(const Mat &pos_in_ptz, float buff_angle);
    void get_Angel_Buff(const Mat &pos_in_ptz, float buff_angle);
    float getBuffPitch(float dist, float tvec_y, float ballet_speed);
    //小孔成像
    float Pinhole_imaging(RotatedRect rects, float _h);
    Mat camera_ptz(Mat &t);
    // void get_Angle(const Mat & pos_in_ptz);
    Point armor_point;
    vector<Point2f> target2d;
    vector<Point3f> object_3d;
    Mat cameraMatrix, distCoeffs, rotM, rotT;
    Mat rvec = Mat::zeros(3, 3, CV_64FC1);
    Mat tvec = Mat::zeros(3, 1, CV_64FC1);
    // Mat rvec_invert = Mat::zeros(3, 3, CV_64FC1);
    // Mat world_point = Mat::zeros(3, 1, CV_8UC1);
    
    float angle_x, angle_y, dist;
    const float ptz_camera_x = PTZ_CAMERA_X;
    const float ptz_camera_y = PTZ_CAMERA_Y;
    const float ptz_camera_z = PTZ_CAMERA_Z;
    const float barrel_ptz_offset_x = 0;
    const float barrel_ptz_offset_y = 0;
    const float overlap_dist = 0;

};