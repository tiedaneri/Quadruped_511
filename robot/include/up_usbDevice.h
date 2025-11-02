/*
 * @Author: your name
 * @Date: 2025-09-03 16:16:09
 * @LastEditTime: 2025-09-18 11:14:15
 * @LastEditors: Please set LastEditors
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
    float sdata[6];       //角位移传感器+力传感器
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
    vector<uint8_t> _rx_buffer;         // 用于缓存从串口读取的数据

    uint8_t open_port(void);
    uint8_t configure_port(void);
};



#endif /* USB_DEVICE_H */