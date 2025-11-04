/*
 * @Author: your name
 * @Date: 2025-09-09 16:09:26
 * @LastEditTime: 2025-11-04 11:51:37
 * @LastEditors: Please set LastEditors
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
    memset(&dataLast, 0, sizeof(dataLast));
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

    // 设置为原始数据模式 (raw mode)
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;

    // 8位数据位, 无校验, 1位停止位
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    
    // 关闭硬件流控
    options.c_cflag &= ~CRTSCTS;
    // 使能接收
    options.c_cflag |= CREAD | CLOCAL;

    // 设置读取超时
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 1; // 0.1秒超时

    tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        printf("[ERROR] usb_device: tcsetattr failed\n");
        return 0;
    }

    return 1;    
}

void usb_device::forceY_fixed(){
    /**
     * Y方向力传感器实时校准函数
     * 
     * 问题描述：
     *   刚开始我没有施加外力，我期望其数据应该在0N左右，但是其数据维持在16N左右（需要解决），
     *   随着我施加正方向的外力，其慢慢增加至19N随后跳变至5N；之后我慢慢卸力，其变化与上边描述相反。
     *   也就是其实际应该输出0-5N的时候输出变为了16-19N
     * 
     * 解决方案：
     *   1. 检测相邻数据点的大幅跳变（>10N阈值）
     *   2. 根据跳变方向累积调整偏移量
     *   3. 减去基线偏移，将数据校准到真实的0N起点
     * 
     * 注意：此函数每个控制周期调用一次，只处理sdata[4]（Y方向力）
     */
    
    // 当前周期的Y方向力（已经过初步转换）
    float currentForceY = data.sdata[4];
    
    // 第一次调用时，初始化上一周期数据
    if (!forceY_initialized) {
        forceY_initialized = true;
        // 第一次不做跳变检测，只应用基线校准
        data.sdata[4] = currentForceY - forceY_baseline_offset;
        return;
    }
    
    // 上一周期的Y方向力（需要恢复到原始未校准的值）
    // dataLast.sdata[4] 是校准后的值，需要加回基线偏移和上一周期的累积偏移
    float lastForceY = dataLast.sdata[4] + forceY_baseline_offset - forceY_cumulative_offset_last;
    
    // 计算相邻周期的力值差
    float diff = currentForceY - lastForceY;
    
    // 步骤1：检测并修复跳变
    
    // 检测向下跳变（如19N -> 5N，差值约-14N）
    if (diff < -forceY_jump_threshold) {
        // 累积偏移增加一个模数
        forceY_cumulative_offset += forceY_modulo;
        
        // 调试输出（建议在调试阶段启用）
        printf("[forceY_fixed] 检测到向下跳变: %.3f -> %.3f (diff=%.3f), 累积偏移=%.3f\n", 
               lastForceY, currentForceY, diff, forceY_cumulative_offset);
    }
    // 检测向上跳变（如5N -> 19N，在卸力阶段）
    else if (diff > forceY_jump_threshold) {
        // 累积偏移减少一个模数
        forceY_cumulative_offset -= forceY_modulo;
        
        // 调试输出（建议在调试阶段启用）
        printf("[forceY_fixed] 检测到向上跳变: %.3f -> %.3f (diff=%.3f), 累积偏移=%.3f\n", 
               lastForceY, currentForceY, diff, forceY_cumulative_offset);
    }    

    // 步骤2：应用累积偏移量
    float fixedForceY = currentForceY + forceY_cumulative_offset;    

    // 步骤3：校准数据（减去基线偏移）
    data.sdata[4] = fixedForceY - forceY_baseline_offset;    

    // 步骤4：保存当前周期的累积偏移，供下一周期使用
    forceY_cumulative_offset_last = forceY_cumulative_offset;
    
    // 可选：限幅保护，防止异常值
    // 根据实际应用场景，可以添加合理的力值范围限制
    // 例如：if (data.sdata[4] < -10.0f) data.sdata[4] = -10.0f;
    //      if (data.sdata[4] > 30.0f) data.sdata[4] = 30.0f;
}

bool usb_device::read_and_parse_data(){
    /* 0. 先存上一个周期的数据（包含校准后的Y方向力） */
    memcpy(&dataLast, &data, sizeof(usb_device_data));

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
                    
                    /* 3. 修复力值跳变 */
                    forceY_fixed();

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