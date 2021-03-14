#ifndef CONTROL_H
#define CONTROL_H
#define ARC_ANGLE 57.29578
/**
 * @brief 弧度转角度
 * 
 */
#define CALL_SERIALPORT 1
/**
 * @brief 打开串口传输
 * @param: 0   不使用
 * @param: 1   使用
 */
#define SERIAL_COMMUNICATION_PLAN 1
/**
  @brief: 串口所发送的方案
  @param: 0         二维＋深度
  @param: 1         云台俯仰与偏航角度
*/
#define SHOW_SERIAL_INFORMATION 0
/**
  @brief: 是否打印串口数据信息
  @param: 0     不打印
  @param: 1     打印
*/
/*---------------------------------------------------*/

#define CALL_KALMAN 0
/**
 * @brief 是否调用卡尔曼
 * @param: 1 调用
 * @param: 0 不调用
 */
#define CALL_DEPTH_INFORMATION 1
/**
 * @brief 是否调用深度信息
 * @param: 1 调用
 * @param: 0 不调用
 */
#define USB_CAPTURE_DEFULT "/home/xx/下载/视频/效果图/armor_2.avi"
/**
  @brief: 相机的默认值
  @note: 使用普通USB相机时，Opencv的VideoCapture接口的值
*/

#define ISOPEN_INDUSTRY_CAPTURE 0
/**
  @brief: 是否使用工业相机
  @param: 0     使用工业相机
  @param: 1     使用普通USB相机
*/
#define MAXIMUM_LOSS 2
/**
 * @brief 最大丢失次数
 * 
 */
#define ROI_IMG 1
/**
 * @brief 打开ROI截取
 * @param 1 截取
 * @param 0 不截取 
 */
#define FPS_SHOW 0
/**
 * @brief 是否显示帧数
 * @param 1 显示
 * @param 0 不显示
 */

#define DRAW_LIGHT_IMG 0
/**
 * @brief 是否绘制灯条
 * @param 1 绘制
 * @param 0 不绘制
 */

#define DRAW_ARMOR_IMG 1
/**
 * @brief 是否绘制装甲板在图像上
 * @param 1 绘制
 * @param 0 不绘制
 */

#define SHOW_BIN_IMG 0
/**
 * @brief 显示最终得到的二值化图片 
 * @param 1 显示
 * @param 0 不显示
 */

#define ENEMY_COLOR 0
/**
 * @brief 敌方颜色
 * @param 1 红色
 * @param 0 蓝色
 */

#define IS_PARAM_ADJUSTMENT 1
/**
  @brief 是否进入调参模式
  @param 0     否
  @param 1     是
*/

#define SMALL_ARMORPLATE_WIDTH 140
/**
 * @brief 小装甲板实际宽度(mm)
 * 
 */
#define ARMORPLATE_HIGHT 60
/**
 * @brief 装甲板实际高度(mm)
 * 
 */
#define BIG_ARMORPLATE_WIDITH 250
/**
 * @brief 大装机板实际宽度(mm)
 * 
 */

#define FOLCAL_LENGTH_X 485.33929
/**
 * @brief X轴焦距
 * 
 */
#define FOLCAL_LENGTH_Y 485.30742
/**
 * @brief Y轴焦距
 * 
 */
#define CALL_PNP 0
/**
 * @brief 是否调用测距
 * @param 0     否
 * @param 1     是
 */

#define LIGHT_WIDITH 10
/**
 * @brief 灯条实际宽度(mm)
 * 
 */
#define CAMERA_EXPOSURETIME 400
#define CAMERA_RESOLUTION_COLS 1280
#define CAMERA_RESOLUTION_ROWS 800
#define CAMERA_RESOLUTION_COLS_FOV ((1280 - CAMERA_RESOLUTION_COLS) * 0.5)
#define CAMERA_RESOLUTION_ROWS_FOV ((1024 - CAMERA_RESOLUTION_ROWS) * 0.5)
#define CAMERA_RED_GAIN 100
#define CAMERA_GREEN_GAIN 100
#define CAMERA_BLUE_GAIN 100
/**
  @brief: 设置相机的分辨率
  @param: CAMERA_EXPOSURETIME   相机曝光时间
  @param: COLS                  为图像的宽度
  @param: ROWS                  为图像的高度
  @param: FOV                   为图像对应左上角的偏移值
  @note: 这部分相机文档中是写反的　x轴　和　y轴
         偏移值计算为 *** (相机最大分辨率 - 当前设置分辨率)/2 ***
*/
/*---------------------------------------------------*/

#if ISOPEN_INDUSTRY_CAPTURE == 1
#define CAMERA_PARAM_FILE "/home/xx/github/armorplate/camera.xml"

#elif ISOPEN_INDUSTRY_CAPTURE == 0
#define CAMERA_PARAM_FILE "/home/gcurobot/armorplate/camera.xml"
#endif
/**
 * @brief 相机标定文件位置
 * 
 */
#define PI 3.1415926
/**
 * @brief π
 * 
 */
// #define BIG_ARMORPLATE_WIDTH 25
// /**
//  * @brief 大装甲板宽度
//  * 
//  */
#define CAMERA_HEIGHT 700
/**
 * @brief 相机高度
 * 
 */
//buff-filter(buff_detect.cpp)
#define SHOW_ANGLE_INFORMATION 1
/**
  @brief: 是否打印PNP解算角度信息
  @param: 0     不打印
  @param: 1     打印
*/
#define REVISE 0.1

#define MAX_BUFF_WIDTH 230
#define MAX_BUFF_HEIGHT 140
#define MAX_BUFF_RADIUS 700

#define ANTI_RANGE 1.01 //指数增长的底数
#define ACC 0.000001f
#define MNC 0.0000000001f //测量协方差矩阵R，更大会有更慢的回归
#define DEAD_BAND 0
#define SIZE_X 960
#define SIZE_Y 480

//buff-model尺寸(solve_pnp.cpp)
#define BULLET_SPEED 29             //子弹射速
#define BUFF_BOTTOM_H -100          //buff最底装甲板距离地面高度 728.84
#define ROBOT_H 330                 //枪口高度    现在是330~340 也有可能是摄像头高度，待测
#define BUFF_ROBOT_Z 6915.340249311 //枪口和buff的直线距离    6915.340249311 6817.708 7212.708
#define OFFSET_Y_BARREL_PTZ 0       //枪管和云台的高度差

#define PTZ_CAMERA_X 0.f
#define PTZ_CAMERA_Y 45.5
#define PTZ_CAMERA_Z 68.9

//启用pid修正
#define PID
// pid修正参数
#define WIDTH 640
#define HEIGHT 480

#define KP 0.6
#define KI 0.02
#define KD 0.1

#define PRE_ANGLE 35
#endif