/**
 * @file rm_VideoCap.h
 * @author GCUROBOT-Visual-Group (GCUROBOT_WOLF@163.com)
 * @brief RM 2019 步兵视觉相机配置头文件
 * @version 1.1
 * @date 2019-05-06
 * @copyright Copyright (c) 2019 GCU Robot Lab. All Rights Reserved.
 */
#ifndef VideoCap_H
#define VideoCap_H

#include "configure.h"
#include "control.h"

class VideoCap
{
public:
    unsigned char *g_pRgbBuffer; //处理后数据缓存区
    int iCameraCounts = 1;
    int iStatus = -1;
    tSdkCameraDevInfo tCameraEnumList;
    int hCamera;
    tSdkCameraCapbility tCapability; //设备描述信息
    tSdkFrameHead sFrameInfo;
    BYTE *pbyBuffer;
    IplImage *iplImage = nullptr;
    int channel = 3;
    BOOL AEstate = FALSE;
    tSdkImageResolution pImageResolution; //相机分辨率信息
    bool iscamera0_open = false;

    VideoCap(int cameramode);
    ~VideoCap();
    bool isindustryimgInput();
    void cameraReleasebuff();
    int cameraSet();
};

#endif // RM_VideoCap_H
