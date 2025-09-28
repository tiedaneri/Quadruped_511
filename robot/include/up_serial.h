/*
 * @Author: sun
 * @Date: 2020-12-14 18:12:39
 * @LastEditTime: 2021-03-19 02:29:46
 * @LastEditors: Please set LastEditors
 * @Description: upboard的串口功能驱动头文件
 * @FilePath: /robot_software/robot/include/up_serial.h
 */
/*
指南:
      0. 创建对象时自动运行: up_serial()
      1. 设置串口参数变量: port, baud
         "up_serial()" 函数已经设置了默认值
      2. 打开串口: open_port();
      3. 配置成sbus模式: sbus_mode();
      4. 读取一个字节: read_byte();
      5. 读取的数据保存到变量: receive_byte
*/
#ifndef UP_SERIAL_H
#define UP_SERIAL_H

//<asm-generic/termbits.h>中的termios与
//lordIMU中包含的termios重复，此处通过宏定义避免重复
#define termios asmtermios
//包含"termios2"的定义
#include <asm-generic/termbits.h>
//取消termios的宏定义，防止对其他地方的termios产生影响
#undef termios

#include <stdio.h> //包含"printf"函数
#include <stdint.h>//包含"uint8_t"定义
#include <string>  //包含"std::string"定义

/*
串口功能驱动类
*/
class up_serial
{
  public:
    up_serial(void);

    uint8_t open_port(void);
    uint8_t sbus_mode(void);
    uint8_t read_byte(void);

    //变量：存储串口端口名称
    std::string port;
    //变量：存储串口端口波特率参数
    uint32_t baud;
    //变量：存储收到的数据
    uint8_t receive_byte;

  private:
    //变量：存储串口的端口ID
    int fd;
    //变量：存储串口配置信息
    termios2 tty;
};

#endif // #ifndef UP_SERIAL_H
