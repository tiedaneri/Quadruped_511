/*
 * @Author: your name
 * @Date: 2025-09-09 16:09:26
 * @LastEditTime: 2025-11-11 14:58:34
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Description: In User Settings Edit
 * @FilePath: /robot_software_WBC/robot/source/up_usbDevice.cpp
 */
#include "up_usbDevice.h"
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

usb_device::usb_device()
{
    port = "/dev/ttyUSB0";  //传感器采集板连接的设备节点
    baud = 460800;          //波特率
    fd = -1;
    memset(&data, 0, sizeof(data));
}

usb_device::~usb_device(){
    if(fd >= 0){
        close(fd);
    }
}

uint8_t usb_device::init_usb(){
    if(!open_port()){
        printf("[Error] usb_device: failed to open port %s\n", port.c_str());
        return 0;
    }

    if(!configure_port()){
        printf("[Error] usb_device: failed to configure port %s\n", port.c_str());
        return 0;
    }

    printf("[usb_device] initialized successfully on %s\n", port.c_str());
    return 1;
}

uint8_t usb_device::open_port(){
    fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if(fd < 0){
        return 0;
    }
    // 将文件描述符设置为阻塞模式，直到有数据可读
    fcntl(fd, F_SETFL, 0);
    return 1;
}

uint8_t usb_device::configure_port(){
    if(tcgetattr(fd, &options) != 0){
        printf("[ERROR] usb_device: tcgetattr failed\n");
        return 0;
    }

    // 设置波特率
    cfsetospeed(&options, B460800);
    cfsetispeed(&options, B460800);

    //关闭流控/输入转换
    options.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR | IGNCR | BRKINT | PARMRK | ISTRIP);
    options.c_oflag &= ~OPOST;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG | IEXTEN);
    options.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
    options.c_cflag |= (CS8 | CREAD | CLOCAL);

    // 设置读取超时
    options.c_cc[VMIN]  = sizeof(usb_device_data); // 至少一帧长度再返回
    options.c_cc[VTIME] = 2; // 0.2s 超时

    tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        printf("[ERROR] usb_device: tcsetattr failed\n");
        return 0;
    }

    return 1;    
}

bool usb_device::read_and_parse_data(){

    /* 1. 从串口读取数据，追加到缓存区 */
    uint8_t temp_buf[256];
    int bytes_read = read(fd, temp_buf, sizeof(temp_buf));
    if (bytes_read > 0) {
        _rx_buffer.insert(_rx_buffer.end(), temp_buf, temp_buf + bytes_read);
    } else {
        // 读取超时或出错     
        return false;
    }

    /* 2. 在缓存区中查找有效的数据帧 */
    const uint8_t expected_header[] = {0xAA, 0x55, 0x18, 0x00}; // 示例帧头
    const uint8_t expected_tail[] = {0x00, 0x00, 0x80, 0x7f};   // 示例帧尾
    const size_t packet_size = sizeof(usb_device_data);

    // 循环查找，直到找不到更多完整的帧
    while(_rx_buffer.size() >= packet_size){
        bool found_packet = false;
        for(size_t i = 0; i <= _rx_buffer.size() - packet_size; ++i){
            // 检查帧头
            if(memcmp(&_rx_buffer[i], expected_header, sizeof(expected_header)) == 0){
                // 检查帧尾
                if(memcmp(&_rx_buffer[i + sizeof(usb_device_data) - sizeof(expected_tail)], expected_tail, sizeof(expected_tail)) == 0){
                    // 找到了一个完整的有效帧
                    memcpy(&data, &_rx_buffer[i], packet_size);

                    // 将raw_data处理为角度和力
                    data.sdata[1] = (data.sdata[1] - angleY_0)/factorY; // Y角度
                    data.sdata[2] = (data.sdata[2] - angleZ_0)/factorZ; // Z角度

                    data.sdata[3] = -(data.sdata[3] - ref_voltageX) * 271.58f; // X方向力 (N)
                    data.sdata[4] = -(data.sdata[4] - ref_voltageY) * 299.58f; // Y方向力 (N)
                    data.sdata[5] = -(data.sdata[5] - ref_voltageZ) * 305.25f; // Z方向力 (N)
                    
                    // 从缓存区中删除已处理的数据（包括该帧本身和之前的所有无效数据）
                    _rx_buffer.erase(_rx_buffer.begin(), _rx_buffer.begin() + i + packet_size);

                    found_packet = true;
                    return true; // 成功解析
                }
            }
        }
        
        if (!found_packet) {
            // 如果循环完整个缓冲区都找不到一个有效的帧头，说明缓冲区头部是无效数据
            // 删除部分旧数据以防缓冲区无限增长，保留一部分可能包含部分帧的数据
            if(_rx_buffer.size() > packet_size) {
                _rx_buffer.erase(_rx_buffer.begin(), _rx_buffer.begin() + _rx_buffer.size() - packet_size + 1);
            }
            break; // 退出while循环
        }
    }

    return false; // 未找到完整数据帧
}