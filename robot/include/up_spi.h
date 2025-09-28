/*
 * @Author: sun
 * @Date: 2020-12-20 05:59:24
 * @LastEditTime: 2021-03-19 02:44:30
 * @LastEditors: Please set LastEditors
 * @Description: upboard的spi功能驱动头文件
 * @FilePath: /robot_software/robot/include/up_spi.h
 */
/*
指南:
      0. 创建对象时自动运行: up_spi();
      1. 设置spi参数变量: port, mode...
         "up_spi()" 函数已经设置了默认参数
      2. 打开端口: open_port();
      3. 配置参数: set_para();
      4. 执行收发通信: send_receive(uint16_t* tx, uint16_t* rx, uint32_t word_length_tx, uint32_t word_length_rx);
*/

#ifndef UP_SPI_H
#define UP_SPI_H

#include <stdio.h> //包含"printf"函数
#include <stdint.h>//包含"uint8_t"定义
#include <string>  //包含"std::string"定义
//包含spi功能函数及其参数
#include <linux/spi/spidev.h>

/*
spi功能驱动类
*/
class up_spi
{
  public:
    up_spi(void);

    uint8_t open_port(void);
    uint8_t set_para(void);
    uint8_t send_receive(uint16_t* tx, uint16_t* rx, uint32_t word_length_tx, uint32_t word_length_rx);//do spi communication, send and receive data

    //变量：存储spi端口名称
    std::string port;
    //变量：存储spi模式参数
    uint8_t mode;
    //变量：存储spi每次发送数据的bits
    uint8_t bits;
    //变量：存储spi是否从低位开始收发的参数
    uint8_t lsb;
    //变量：存储spi通信速率
    uint32_t speed;
    //变量：存储spi的CS引脚变化参数
    uint8_t cs_change;
    //变量：存储spi延时参数，单位：us
    uint8_t delay_usecs;

  private:
    //变量：存储spi端口的ID
    int fd;
    //变量：spi信息结构体变量
    spi_ioc_transfer spi_message[1];

};

#endif // #ifndef UP_SPI_H
