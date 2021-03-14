/**
 * @file VideoCap.cpp
 * @author GCUROBOT-Visual-Group (GCUROBOT_WOLF@163.com)
 * @brief RM 2019 步兵视觉相机配置部分源文件
 * @version 2.0
 * @date 2019-05-06
 * @copyright Copyright (c) 2019 GCU Robot Lab. All Rights Reserved.
 */
#include "videocapture.h"

/**
 * @brief Construct a new rm VideoCap::rm VideoCap object
 * @param cameramode 相机型号
 */
VideoCap::VideoCap(int cameramode)
{
    if (cameramode == 0)
    {
        this->cameraSet();
        this->iscamera0_open = true;
        cout << "Set camera Industrial camera" << endl;
    }
    else
    {
        iscamera0_open = false;
        cout << "set camera USB camera" << endl;
    }
}

/**
 * @brief Destroy the rm VideoCap::rm VideoCap object
 */
VideoCap::~VideoCap()
{
    if (iscamera0_open)
    {
        CameraUnInit(hCamera);
        //注意，现反初始化后再free
        free(g_pRgbBuffer);
        cout << "release Industry camera success......" << endl;
    }
    else
    {
        cout << "release USB camera success......" << endl;
    }
}

/**
 * @brief 工业相机的图像转到指针中 再通过指针转换变为 Mat
 * @return true 成功启用工业相机
 * @return false 不能启用工业相机
 */
bool VideoCap::isindustryimgInput()
{
    bool isindustry_camera_open = false;
    if (iscamera0_open == 1)
    {
        if (CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 1000) == CAMERA_STATUS_SUCCESS)
        {
            //----------读取原图----------//
            CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer, &sFrameInfo);
            if (iplImage)
            {
                cvReleaseImageHeader(&iplImage);
            }
            iplImage = cvCreateImageHeader(cvSize(sFrameInfo.iWidth, sFrameInfo.iHeight), IPL_DEPTH_8U, channel);
            cvSetData(iplImage, g_pRgbBuffer, sFrameInfo.iWidth * channel); //此处只是设置指针，无图像块数据拷贝，不需担心转换效率
        }
        isindustry_camera_open = true;
    }
    else
    {
        isindustry_camera_open = false;
    }
    return isindustry_camera_open;
}

/**
 * @brief 工业相机初始化
 * @return int
 */
int VideoCap::cameraSet()
{
    CameraSdkInit(1);
    //枚举设备，并建立设备列表
    iStatus = CameraEnumerateDevice(&tCameraEnumList, &iCameraCounts);
    printf("state = %d\n", iStatus);
    printf("count = %d\n", iCameraCounts);
    //没有连接设备
    if (iCameraCounts == 0)
    {
        return -1;
    }
    //相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    iStatus = CameraInit(&tCameraEnumList, -1, -1, &hCamera);
    //初始化失败
    printf("state = %d\n", iStatus);
    if (iStatus != CAMERA_STATUS_SUCCESS)
    {
        return -1;
    }
    //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    CameraGetCapability(hCamera, &tCapability);
    g_pRgbBuffer = (unsigned char *)malloc(tCapability.sResolutionRange.iHeightMax * tCapability.sResolutionRange.iWidthMax * 3);
    /*--------设置分辨率---------*/
    CameraGetImageResolution(hCamera, &pImageResolution);
    pImageResolution.iIndex = 0xFF;
    pImageResolution.iWidthFOV = CAMERA_RESOLUTION_COLS;
    pImageResolution.iHeightFOV = CAMERA_RESOLUTION_ROWS;
    pImageResolution.iWidth = CAMERA_RESOLUTION_COLS;
    pImageResolution.iHeight = CAMERA_RESOLUTION_ROWS;
    pImageResolution.iHOffsetFOV = int(CAMERA_RESOLUTION_COLS_FOV);
    pImageResolution.iVOffsetFOV = int(CAMERA_RESOLUTION_ROWS_FOV);
    CameraSetImageResolution(hCamera, &pImageResolution);
    /*--------设置分辨率---------*/

    /*--------设置曝光时间---------*/
    cout << CameraGetAeState(hCamera, &AEstate);
    cout << CameraSetAeState(hCamera, FALSE);
    if (ENEMY_COLOR == 0)
    {
        CameraSetExposureTime(hCamera, CAMERA_EXPOSURETIME);
    }
    else
    {
        CameraSetExposureTime(hCamera, CAMERA_EXPOSURETIME);
    }
    /*--------设置曝光时间---------*/

    /*让SDK进入工作模式，开始接收来自相机发送的图像数据。
     *如果当前相机是触发模式，则需要接收到触发帧以后才会更新图像*/
    CameraPlay(hCamera);
    CameraReleaseImageBuffer(hCamera, pbyBuffer);
    /*
    其他的相机参数设置
    例如 CameraSetExposureTime   CameraGetExposureTime  设置/读取曝光时间
         CameraSetImageResolution  CameraGetImageResolution 设置/读取分辨率
         CameraSetGamma、CameraSetConrast、CameraSetGain等设置图像伽马、对比度、RGB数字增益等等。
         更多的参数的设置方法，，清参考MindVision_Demo。本例程只是为了演示如何将SDK中获取的图像，转成OpenCV的图像格式,以便调用OpenCV的图像处理函数进行后续开发
    */
    if (tCapability.sIspCapacity.bMonoSensor)
    {
        channel = 1;
        CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_MONO8);
    }
    else
    {
        channel = 3;
        CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_BGR8);
    }
    return 1;
}

/**
 * @brief 释放相机缓存数据
 */
void VideoCap::cameraReleasebuff()
{
    if (iscamera0_open)
    {
        //在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
        //否则再次调用CameraGetImageBuffer时，程序将被挂起一直阻塞，直到其他线程中调用CameraReleaseImageBuffer来释放了buffer
        CameraReleaseImageBuffer(hCamera, pbyBuffer);
    }
}
