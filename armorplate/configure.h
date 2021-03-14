#include "control.h"
#include "CameraApi.h"
/*---工业相机中使用到opencv2.0的 IplImage 需要包含此头文件 ---*/
#include "opencv2/imgproc/imgproc_c.h"
/*---工业相机中使用到opencv2.0的 cvReleaseImageHeader 需要包含此头文件 ---*/

/*---- OpenCV header files ----*/
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
/*---- OpenCV header files ----*/

/*---- Others header files ----*/
#include <cmath>
#include <math.h>
#include <iostream>
/*---- Others header files ----*/

/*---- Serial header files ----*/
#include <string.h>
#include <fcntl.h>   //文件控制定义
#include <termios.h> //POSIX终端控制定义
#include <unistd.h>  //UNIX标准定义
#include <errno.h>   //ERROR数字定义
#include <sys/select.h>
/*---- Serial header files ----*/
using namespace std;
using namespace cv;