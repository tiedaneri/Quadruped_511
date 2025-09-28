/*
 * @Author: sun
 * @Date: 2020-12-14 18:12:39
 * @LastEditTime: 2021-03-19 02:30:04
 * @LastEditors: Please set LastEditors
 * @Description: upboard的串口功能驱动源文件
 * @FilePath: /robot_software/robot/source/up_serial.cpp
 */

#include "up_serial.h"

#include <string.h> //包含"string" 函数
#include <fcntl.h>  //包含"open" 函数及其参数
#include <unistd.h> //包含"read"函数
#include <stropts.h>//包含"ioctl" 函数
//包含"ioctl"函数的参数
#include <asm-generic/ioctls.h>

/**
 * @description: 构造函数，初始化参数
 * @param {无}
 * @return {无}
 */
up_serial::up_serial()
{
    port = "/dev/ttyS4";
    baud = 100000;
}

/**
 * @description: 打开串口
 * @param {无}
 * @return {0: 失败; 1: 成功}
 */
uint8_t up_serial::open_port()
{
    //打开串口
    fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if(fd < 0)//失败
    {
        printf("[ERROR] open serial port: %s\n", port.c_str());
        return 0;//失败
    } 
    return 1;//成功
}

/**
 * @description: 将串口把配置成sbus模式
 * @param {无}
 * @return {0: 失败; 1: 成功}
 */
uint8_t up_serial::sbus_mode()
{
    if(ioctl(fd, TCGETS2, &tty) < 0)
    {
        printf("[ERROR] set sbus mode: TCGETS2: %s\n", port.c_str());
        return 0;//失败
    }
    tty.c_cflag &= ~CBAUD;
    tty.c_cflag |= BOTHER;
    tty.c_ispeed = baud;
    tty.c_ospeed = baud;
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);             
    tty.c_oflag &= ~OPOST;
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_cflag &= ~(CSIZE | PARENB);
    tty.c_cflag |= PARENB;
    tty.c_cflag &= ~PARODD;
    tty.c_cflag |= tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;
    if(ioctl(fd, TCSETS2, &tty) < 0)
    {
        printf("[ERROR] set sbus mode: TCSETS2: %s\n", port.c_str());
        return 0;//失败
    }
    return 1;//成功

}

/**
 * @description: 从串口读取一个字节
 * @param {无}
 * @return {0: 失败; 1: 成功}
 */
uint8_t up_serial::read_byte()
{
    //定义数组用于临时保存读取的数据
    uint8_t read_byte[1] = {0};
    //定义变量用于记录读取数据出错的次数
    uint32_t counter = 0;
    //从串口读取一个数据
    //当读取失败时，继续循环读取多次
    while(read(fd, read_byte, sizeof(read_byte)) != 1)
    {
        counter++;//失败次数加1
        if(counter >= 0xF) return 0;//失败次数超限，放弃读取，返回失败值
    }
    //读取成功
    receive_byte = read_byte[0];//保存读取到的数据
    return 1;//成功

}
