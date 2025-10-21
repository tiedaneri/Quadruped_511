/*
 * @Author: sun
 * @Date: 2020-12-14 06:21:51
 * @LastEditTime: 2024-05-23 06:24:41
 * @LastEditors: Please set LastEditors
 * @Description: 读取遥控器（at9s）的数据，并解析成指令
 * @FilePath: /robot_software/robot/source/up_at9s.cpp
 */

#include "up_at9s.h"

#include <string.h>//包含"memset"定义

/**
 * @description: 构造函数，初始化参数
 * @param {无}
 * @return {无}
 */
up_at9s::up_at9s():up_serial()//初始化基类
{
    memset(&cmd, 0, sizeof(cmd));
    memset(&original_data, 0, sizeof(original_data));
    memset(&channel_data, 0, sizeof(channel_data));
}

/**
 * @description: 初始化串口，打开串口并配置成sbus模式
 * @param {无}
 * @return {0: 失败; 1: 成功}
 */
uint8_t up_at9s::init_serial()
{
    port = "/dev/ttyS4";//端口
    baud = 100000;//波特率
    if(open_port())//打开串口: "/dev/ttyS4"
    {
        sbus_mode();//设置为sbus模式
        return 1;//成功
    }
    else return 0;//失败
}

/**
 * @description: 接收一个原始数据包
 * @param {无}
 * @return {0: failed; 1: success}
 */
uint8_t up_at9s::receive_original_data()
{
    //定义变量表示接收完整数据包标志，1表示接收完整数据包
    uint8_t packet_flag = 0;
    int byte_counter = 0;//记录接收的字节数
    while (!packet_flag)//循环接收数据，直到接收到完整数据包
    {
        byte_counter++;//接收字节数加1
        //存储接收数据的数组数据左移一个字节
        for (int i = 0; i < 24; i++) original_data[i] = original_data[i+1];
        //接收一个字节
        if(!read_byte()){
            return 0;//接收失败则退出
        }
        
        //成功接收一个字节，将其放到接收数组的最后一个位置
        original_data[24]=receive_byte;
        //当存储接收数据的数组的第一个字节和最后一个字节分别跟与包头包尾相同时，表示接收到一个完整的数据包
        //将完整数据包标志置1
        if ((original_data[0] == 0xF) && (original_data[24] == 0x0)) packet_flag = 1;
        //当接收到了50个字节数据，仍没有接收到完整数据包时，表示数据出错
        if (byte_counter >= 50) 
        {
            return 0;//数据出错则退出
        }
    }
    return 1;//成功
}

/**
 * @description: 将原始数据解码为channel数据
 * @param {无}
 * @return {无}
 */
void up_at9s::unpack_original_data()
{
    channel_data[0]  = ((original_data[1])     | ((original_data[2] & 0x7)    << 8));
    channel_data[1]  = (original_data[2] >> 3) | ((original_data[3] & 0x3F)   << 5);
    channel_data[2]  = ((original_data[3]  & 0xC0) >> 6) | (original_data[4]  << 2) | ((original_data[5] & 0x1)  << 10);
    channel_data[3]  = ((original_data[5]  & 0xFE) >> 1) | ((original_data[6]  & 0xF)  << 7);
    channel_data[4]  = ((original_data[6]  & 0xF0) >> 4) | ((original_data[7]  & 0x7F) << 4);
    channel_data[5]  = ((original_data[7]  & 0x80) >> 7) | (original_data[8]  << 1) | ((original_data[9] & 0x3)  << 9);
    channel_data[6]  = ((original_data[9]  & 0xFC) >> 2) | ((original_data[10] & 0x1F) << 6);
    channel_data[7]  = ((original_data[10] & 0xE0) >> 5) | (original_data[11] << 3);
    channel_data[8]  = ((original_data[12])     | ((original_data[13] & 0x7)  << 8));
    channel_data[9]  = (original_data[13] >> 3) | ((original_data[14] & 0x3F) << 5);
    channel_data[10] = ((original_data[14] & 0xC0) >> 6) | (original_data[15] << 2) | ((original_data[16] & 0x1) << 10);
    channel_data[11] = ((original_data[16] & 0xFE) >> 1) | ((original_data[17] & 0xF)  << 7);
    channel_data[12] = ((original_data[17] & 0xF0) >> 4) | ((original_data[18] & 0x7F) << 4);
    channel_data[13] = ((original_data[18] & 0x80) >> 7) | (original_data[19] << 1) | ((original_data[20] & 0x3) << 9);
    channel_data[14] = ((original_data[20] & 0xFC) >> 2) | ((original_data[21] & 0x1F) << 6);
    channel_data[15] = ((original_data[21] & 0xE0) >> 5) | (original_data[22] << 3);
    channel_data[16] = (original_data[23]  & 0x80) >> 7;
    channel_data[17] = (original_data[23]  & 0x40) >> 6;
}

/**
 * @description: 将channel数据解析成遥控器指令
 * @param {无}
 * @return {无}
 */
void up_at9s::analysis_cmd()
{
    if(channel_data[6] == 306)  cmd.SWA = UP;
    if(channel_data[6] == 1694) cmd.SWA = DOW;

    if(channel_data[9] == 306)  cmd.SWD = UP;
    if(channel_data[9] == 1694) cmd.SWD = DOW;

    if(channel_data[8] == 306)  cmd.SWC = UP;
    if(channel_data[8] == 1000) cmd.SWC = MID;
    if(channel_data[8] == 1694) cmd.SWC = DOW;

    if(channel_data[4] == 1694) cmd.SWE = UP;
    if(channel_data[4] == 1000) cmd.SWE = MID;
    if(channel_data[4] == 306)  cmd.SWE = DOW;

    if(channel_data[5] == 1694) cmd.SWG = UP;
    if(channel_data[5] == 1000) cmd.SWG = MID;
    if(channel_data[5] == 306)  cmd.SWG = DOW;

    cmd.LRO = (996.0f  - (float)channel_data[0]) / 689.0f / 2.0f;
    cmd.RRO = (1006.0f - (float)channel_data[3]) / 672.0f / 2.0f;
    cmd.LCO = (1006.0f - (float)channel_data[1]) / 570.0f / 2.0f;
#ifdef ROBOT1
    cmd.RCO = ((999.0f  - (float)channel_data[2]) / 607.0f / 2.0f) - 0.253f; //遥控器1的vx指令不准，加偏置
#endif
#ifdef ROBOT2
    cmd.RCO = (999.0f  - (float)channel_data[2]) / 607.0f / 2.0f;
#endif
    cmd.VB = (1694.0f  - (float)channel_data[7]) / 1388.0f;
}

/**
 * @description: 应用死区屏蔽遥控器零飘
 * @param {无}
 * @return {无}
 */
void up_at9s::applyDeadZone()
{
    if(fabsf(cmd.LRO) <= DEADZONE_THRESHOLD) cmd.LRO = 0.0f;  // 左摇杆横向
    if(fabsf(cmd.LCO) <= DEADZONE_THRESHOLD) cmd.LCO = 0.0f;  // 左摇杆竖向  
    if(fabsf(cmd.RRO) <= DEADZONE_THRESHOLD) cmd.RRO = 0.0f;  // 右摇杆横向
    if(fabsf(cmd.RCO) <= DEADZONE_THRESHOLD) cmd.RCO = 0.0f;  // 右摇杆竖向
    if(fabsf(cmd.VB)  <= DEADZONE_THRESHOLD) cmd.VB  = 0.0f;  // 旋钮VB
}

/**
 * @description: 接收遥控器数据并解析成指令
 * @param {无}
 * @return {0: 失败; 1: 成功}
 */
uint8_t up_at9s::update_cmd()
{
    if(!receive_original_data()) return 0;//接收原始数据，出错则退出
    unpack_original_data();//解码原始数据
    analysis_cmd();//解析指令
    applyDeadZone();
    return 1;//成功
}
