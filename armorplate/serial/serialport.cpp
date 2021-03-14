/**
 * @file serialport.cpp
 * @author GCUROBOT-Visual-Group (GCUROBOT_WOLF@163.com)
 * @brief
 * @version 1.0
 * @date 2019-11-19
 *
 * @copyright Copyright (c) 2019 GCU Robot Lab. All Rights Reserved.
 *
 */
#include "serialport.h"

int SerialPort::fd;
unsigned char SerialPort::g_write_buf[WRITE_BUFF_LENGTH];
unsigned char SerialPort::g_CRC_buf[CRC_BUFF_LENGTH];
unsigned char SerialPort::g_rec_buf[REC_BUFF_LENGTH];

int16_t SerialPort::_yaw_reduction = 0x0000;
int16_t SerialPort::_pitch_reduction = 0x0000;
int16_t SerialPort::_depth_reduction = 0x0000;
/**
* @brief Construct a new Serial Port:: Serial Port object
* ------------------------------------------------------
* @param:  波特率,默认为115200 
* --------------------------------------------------------
* @param:  char parity 是否进行奇偶校验,'Y'表示需要奇偶校验,'N'表示不需要奇偶校验
* -------------------------------------------------------------
* @param:  int databits 数据位的个数,默认值为8个数据位
*----------------------------------------------------------
* @return: bool  初始化是否成功
* @note:   在使用其他本类提供的函数前,请先调用本函数进行串口的初始化
*　　　　　   函数提供了一些常用的串口参数设置
*           本串口类析构时会自动关闭串口,无需额外执行关闭串口
* @author: Hzkkk
*          Rcxxx (revised)
*/
SerialPort::SerialPort()
{
    cout << "The Serial set ......" << endl;
    const char *DeviceName[4] = {"", "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2"};

    /* WARNING :  终端设备默认会设置为控制终端，因此open(O_NOCTTY不作为控制终端)
     * Terminals'll default to be set as Control Terminals
     */
    struct termios newstate;
    /*打开串口*/
    bzero(&newstate, sizeof(newstate)); //清零
    for (size_t i = 0; i < (sizeof(DeviceName) / sizeof(char *)); ++i)
    {
        fd = open(DeviceName[i], O_RDWR | O_NONBLOCK | O_NOCTTY | O_NDELAY);
        if (fd == -1)
        {
            printf("Can't Open Serial Port %s\n", DeviceName[i]);
            continue;
        }
        else
        {
            printf("Open Serial Port %s Successful\n", DeviceName[i]);
            break;
        }
    }
    cfsetospeed(&newstate, B115200);
    cfsetispeed(&newstate, B115200);

    //本地连线, 取消控制功能 | 开始接收
    newstate.c_cflag |= CLOCAL | CREAD;
    //设置字符大小
    newstate.c_cflag &= ~CSIZE;
    //设置停止位1
    newstate.c_cflag &= ~CSTOPB;
    //设置数据位8位
    newstate.c_cflag |= CS8;
    //设置无奇偶校验位，N
    newstate.c_cflag &= ~PARENB;

    /*阻塞模式的设置*/
    newstate.c_cc[VTIME] = 0;
    newstate.c_cc[VMIN] = 0;

    /*清空当前串口*/
    tcflush(fd, TCIOFLUSH);
    tcsetattr(fd, TCSANOW, &newstate);
}

/**
 * @brief Destroy the Serial Port:: Serial Port object
 */
SerialPort::~SerialPort(void)
{
    if (!close(fd))
        printf("Close Serial Port Successful\n");
}

/**
*  @brief: 串口数据读取函数
*  @return: string  返回收到的字符串
*  @note:   逐字节读取并存到字符串
*           等待0.01s后结束读取,将所得字符串返回
*  @authors: Rcxxx
*            Hzkkk
*/
void SerialPort::RMreceiveData(int arr[REC_BUFF_LENGTH])
{
    memset(g_rec_buf, '0', REC_BUFF_LENGTH); //清空缓存
    char rec_buf_temp[REC_BUFF_LENGTH * 2];
    read(fd, rec_buf_temp, sizeof(rec_buf_temp));
    for (int i = 0; i < (int)sizeof(rec_buf_temp); ++i)
    {
        if (rec_buf_temp[i] == 'S' && rec_buf_temp[i + sizeof(g_rec_buf) - 1] == 'E')
        {
            for (int j = 0; j < ((int)sizeof(g_rec_buf)); ++j)
            {
                g_rec_buf[j] = rec_buf_temp[i + j];
            }
            break;
        }
    }
    for (size_t i = 0; i < sizeof(g_rec_buf); ++i)
    {
        arr[i] = (g_rec_buf[i] - '0');
    }
    tcflush(fd, TCIFLUSH);
#if SHOW_SERIAL_INFORMATION == 1
    cout << "  rec_buf_temp: " << rec_buf_temp << endl;
    cout << "  g_rec_buf: " << g_rec_buf << endl;
#endif
}

/**
 *@brief: RM串口发送格式化函数
 *
 * @param: yaw 云台偏航
 * @param: pitch 云台俯仰
 * @param: _yaw yaw正负
 * @param: _pitch pitch正负
 * @param: data_type 是否正确识别的标志
 * @param: data_type 是否正确识别的标志
 *
 * @authors: Rcxxx
 *           Hzkkk
 */
void SerialPort::RMserialWrite(int _yaw, int16_t yaw, int _pitch, int16_t pitch, int16_t depth, int data_type, int is_shooting)
{
    // sprintf(g_CRC_buf, "%c%1d%1d%1d%04d%1d%03d%04d", 'S', data_type, is_shooting, _yaw ,yaw, _pitch, pitch, depth);
    getDataForCRC(data_type, is_shooting, _yaw, yaw, _pitch, pitch, depth);

    uint8_t CRC = Checksum_CRC8(g_CRC_buf, sizeof(g_CRC_buf));
    getDataForSend(data_type, is_shooting, _yaw, yaw, _pitch, pitch, depth, CRC);
    /*
    0：帧头     1：是否正确识别的标志   2：是否射击的信号
    3：yaw正负值    4：yaw低八位数据    5：yaw高八位数据
    6：pitch正负值  7：pitch低八位数据  8：pitch高八位数据
    9：深度低八位   10：深度高八位
    11：CRC
    12：帧尾
    */
    write(fd, g_write_buf, sizeof(g_write_buf));
    memset(g_write_buf,'\0',sizeof(g_write_buf));
#if SHOW_SERIAL_INFORMATION == 1

    _yaw_reduction = (g_write_buf[5] << 8) | _yaw_reduction;
    _yaw_reduction = g_write_buf[4] | _yaw_reduction;

    _pitch_reduction = (g_write_buf[8] << 8) | _pitch_reduction;
    _pitch_reduction = g_write_buf[7] | _pitch_reduction;

    _depth_reduction = (g_write_buf[10] << 8) | _depth_reduction;
    _depth_reduction = g_write_buf[9] | _depth_reduction;

#if SERIAL_COMMUNICATION_PLAN == 1
    cout << "g_write_buf=  " << g_write_buf[0]
         << "  " << static_cast<int>(g_write_buf[1])
         << "  " << static_cast<int>(g_write_buf[2])
         << "  " << static_cast<int>(g_write_buf[3]) << "  " << float(_yaw_reduction) / 100
         << "  " << static_cast<int>(g_write_buf[6]) << "  " << float(_pitch_reduction) / 100
         << "  " << float(_depth_reduction) << "  " << g_write_buf[12] << endl;
#else
    cout << "g_write_buf=  " << g_write_buf[0]
         << "  " << static_cast<int>(g_write_buf[1])
         << "  " << static_cast<int>(g_write_buf[2])
         << "  " << static_cast<int>(g_write_buf[3]) << "  " << float(_yaw_reduction)
         << "  " << static_cast<int>(g_write_buf[6]) << "  " << float(_pitch_reduction)
         << "  " << float(_depth_reduction) << "  " << g_write_buf[12] << endl;
#endif
    _yaw_reduction = 0x0000;
    _pitch_reduction = 0x0000;
    _depth_reduction = 0x0000;

#endif
    //usleep(1);
}
/** CRC8校验函数
 *
 *  @param:  char *buf   需要检验的字符串
 *  @param:  uint16_t len 是否进行奇偶校验,'Y'表示需要奇偶校验,'N'表示不需要奇偶校验
 *
 *  @return: bool  初始化是否成功
 *  @brief:  CRC8校验 ---MAXIM x8+x5+x4+x1  多项式 POLY（Hex）:31(110001)  初始值 INIT（Hex）：00  结果异或值 XOROUT（Hex）：
 *  @note:   在使用其他本类提供的函数前,请先调用本函数进行串口的初始化
 *　　　　　   函数提供了一些常用的串口参数设置
 *           本串口类析构时会自动关闭串口,无需额外执行关闭串口
 */
uint8_t SerialPort::Checksum_CRC8(unsigned char *buf, uint16_t len)
{
    uint8_t check = 0;

    while (len--)
    {
        check = CRC8Tab[check ^ (*buf++)];
    }

    return (check)&0x00ff;
}

void SerialPort::getDataForCRC(int data_type, int is_shooting, int _yaw, int16_t yaw, int _pitch, int16_t pitch, int16_t depth)
{
    g_CRC_buf[0] = 0x53;
    g_CRC_buf[1] = static_cast<unsigned char>(data_type);
    g_CRC_buf[2] = static_cast<unsigned char>(is_shooting);
    g_CRC_buf[3] = static_cast<unsigned char>(_yaw);
    g_CRC_buf[4] = yaw & 0xff;
    g_CRC_buf[5] = (yaw >> 8) & 0xff;
    g_CRC_buf[6] = static_cast<unsigned char>(_pitch);
    g_CRC_buf[7] = pitch & 0xff;
    g_CRC_buf[8] = (pitch >> 8) & 0xff;
    g_CRC_buf[9] = depth & 0xff;
    g_CRC_buf[10] = (depth >> 8) & 0xff;
}

void SerialPort::getDataForSend(int data_type, int is_shooting, int _yaw, int16_t yaw, int _pitch, int16_t pitch, int16_t depth, uint8_t CRC)
{
    g_write_buf[0] = 0x53;
    g_write_buf[1] = static_cast<unsigned char>(data_type);
    g_write_buf[2] = static_cast<unsigned char>(is_shooting);
    g_write_buf[3] = static_cast<unsigned char>(_yaw);
    g_write_buf[4] = yaw & 0xff;
    g_write_buf[5] = (yaw >> 8) & 0xff;
    g_write_buf[6] = static_cast<unsigned char>(_pitch);
    g_write_buf[7] = pitch & 0xff;
    g_write_buf[8] = (pitch >> 8) & 0xff;
    g_write_buf[9] = depth & 0xff;
    g_write_buf[10] = (depth >> 8) & 0xff;
    g_write_buf[11] = CRC & 0xff;
    g_write_buf[12] = 0x45;
}
