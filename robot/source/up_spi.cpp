/*
 * @Author: sun
 * @Date: 2020-12-20 05:59:24
 * @LastEditTime: 2021-03-19 02:31:11
 * @LastEditors: Please set LastEditors
 * @Description: upboard的spi功能驱动源文件
 * @FilePath: /robot_software/robot/source/up_spi.cpp
 */

#include "up_spi.h"

#include <string.h> //包含"string" 函数
#include <fcntl.h>  //包含"open" 函数及其参数
#include <stropts.h>//包含"ioctl" 函数

/**
 * @description: 构造函数，初始化参数
 * @param {无}
 * @return {无}
 */
up_spi::up_spi()
{
    port = "/dev/spidev2.0";
    fd   = -1;
    mode = SPI_MODE_0;
    bits = 8;
    lsb  = 0x01;
    speed = 6000000;
    cs_change = 1;
    delay_usecs = 0;  
}

/**
 * @description: 开启spi端口
 * @param {无}
 * @return {0: 失败; 1: 成功}
 */
uint8_t up_spi::open_port()
{
    fd = open(port.c_str(), O_RDWR);//开启端口
    if(fd < 0)//失败
    {
        printf("[ERROR] up_spi: open spi port: %s\n", port.c_str());
        return 0;//失败
    }
    return 1;//成功
}

/**
 * @description: 配置spi参数
 * @param {无}
 * @return {0: 失败; 1: 成功}
 */
uint8_t up_spi::set_para()
{
    if(ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0)
    {
        printf("[ERROR] up_spi: set SPI_IOC_WR_MODE: %s\n", port.c_str());
        return 0;
    }
    if(ioctl(fd, SPI_IOC_RD_MODE, &mode) < 0)
    {
        printf("[ERROR] up_spi: set SPI_IOC_RD_MODE: %s\n", port.c_str());
        return 0;
    }
    if(ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0)
    {
        printf("[ERROR] up_spi: set SPI_IOC_WR_BITS_PER_WORD: %s\n", port.c_str());
        return 0;
    }
    if(ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits) < 0)
    {
        printf("[ERROR] up_spi: set SPI_IOC_RD_BITS_PER_WORD: %s\n", port.c_str());
        return 0;
    }
    if(ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0)
    {
        printf("[ERROR] up_spi: set SPI_IOC_WR_MAX_SPEED_HZ: %s\n", port.c_str());
        return 0;
    }
    if(ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0)
    {
        printf("[ERROR] up_spi: set SPI_IOC_WR_MAX_SPEED_HZ: %s\n", port.c_str());
        return 0;
    }
    if(ioctl(fd, SPI_IOC_RD_LSB_FIRST, &lsb) < 0)
    {
        printf("[ERROR] up_spi: set SPI_IOC_RD_LSB_FIRST: %s\n", port.c_str());
        return 0;
    }
    memset(spi_message, 0, 1 * sizeof(spi_ioc_transfer));//spi信息结构体数据归零
    spi_message[0].bits_per_word = bits;
    spi_message[0].cs_change = cs_change;
    spi_message[0].delay_usecs = delay_usecs;
    return 1;
}

/**
 * @description: 执行spi通信，收发数据
 * @param {
 *          tx: 发送数据的地址，16bits类型
            rx: 接受数据保存的地址，16bits类型
            word_length_tx: 发送数据的数量，16bits类型
            word_length_rx: 接受数据的数量，16bits类型
          }
 * @return {0: 失败; 1: 成功}
 */
uint8_t up_spi::send_receive(uint16_t* tx, uint16_t* rx, uint32_t word_length_tx, uint32_t word_length_rx)
{
    uint16_t tx_buf[word_length_tx];//定义数组用于临时保存发送数据
    uint16_t rx_buf[word_length_rx];//定义数组用于临时保存接收数据
    //将发送数据保存到临时数据
    for(int i = 0; i < word_length_tx; i++) tx_buf[i] = (tx[i] >> 8) + ((tx[i] & 0xff) << 8);
    //配置spi信息结构体，长度以收发数据中最长的为准，此处配置发送数据大小为8bits，所以长度为16bits长度*2
    spi_message[0].len = word_length_tx > word_length_rx ? word_length_tx * 2 : word_length_rx * 2;
    spi_message[0].rx_buf = (uint64_t)rx_buf;
    spi_message[0].tx_buf = (uint64_t)tx_buf;
    if(ioctl(fd, SPI_IOC_MESSAGE(1),&spi_message) < 0)//执行spi通信
    {
        printf("[ERROR] up_spi: send-receive SPI data: %s\n", port.c_str());
        return 0;//失败
    }
    //通信成功
    //将接收数据从临时数组保存到接收数据地址
    for(int i = 0; i < word_length_rx; i++) rx[i] = (rx_buf[i] >> 8) + ((rx_buf[i] & 0xff) << 8);
    return 1;//成功
}
