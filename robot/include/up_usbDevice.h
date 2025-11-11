/*
 * @Author: your name
 * @Date: 2025-09-03 16:16:09
 * @LastEditTime: 2025-11-11 10:04:28
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Description: In User Settings Edit
 * @FilePath: /robot_software_WBC/robot/include/up_usbDevice.h
 */
#ifndef USB_DEVICE_H
#define USB_DEVICE_H

#include <stdio.h>
#include <stdint.h>
#include <string>
#include <vector>
#include <termios.h>

using namespace std;

struct usb_device_data
{
    uint8_t header[4];
    float sdata[6];
    uint8_t tail[4];
};

class usb_device{
public:
    usb_device(void);
    ~usb_device(void);

    uint8_t init_usb(void);
    bool read_and_parse_data();

    string port;
    uint32_t baud;

    usb_device_data data;               // 解析后数据放在这

private:
    int fd;
    struct termios options;
    vector<uint8_t> _rx_buffer;         // 缓冲区，用于缓存从串口读取的数据

    /* 记录角位移传感器0位和90°位电压 */
    const float angleY_0   = 1.787f;
    const float angleY_m90 = 1.041f;
    const float angleZ_0   = 1.183f;
    const float angleZ_90  = 0.329f;
    const float angleZ_m90 = 2.048f;

    float factorY = (angleY_0 - angleY_m90)/(90.0f);
    float factorZ = (angleZ_0 - angleZ_m90)/(90.0f);

    /* 记录力传感器0位电压 */
    const float ref_voltageX = 2.238f;        // 参考电压 (V)
    const float ref_voltageY = 2.220f;        // 参考电压 (V)
    const float ref_voltageZ = 2.237f;        // 参考电压 (V)

    const float gain_x = 127.34f;           // X轴AD620放大倍数
    const float gain_y = 127.67f;           // Y轴AD620放大倍数
    const float gain_z = 127.67f;           // Z轴AD620放大倍数
    
    uint8_t open_port(void);
    uint8_t configure_port(void);
};



#endif /* USB_DEVICE_H */