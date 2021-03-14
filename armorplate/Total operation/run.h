#include "configure.h"
#include "control.h"
#include "armor/armorplate.h"
#include "pnp/solvepnp.h"
#include "serial/serialport.h"
#include "camera/videocapture.h"
#include "kalmantest/kalmantest.h"
#include "detect_buff/buff_detect.h"
class WorKing
{
public:
    WorKing();
    ~WorKing();
    void Run();              //装甲板
    void ddd();              //拍摄图片
    void Run_MAX_Talisman(); //大神符
    ArmorPlate armor;
    LightBar rgb;
    ImageProcess img;
    Max_Buff buff;
#if CALL_SERIALPORT == 1
    SerialPort serial;
#endif
    SolveP4p pnp;
    cv::VideoCapture capture;
    VideoCap cap;
    RM_kalmanfilter kalman;
    Mat frame;
};
