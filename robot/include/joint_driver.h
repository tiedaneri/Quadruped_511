/*
 * @Author: sun
 * @Date: 2020-12-21 00:03:53
 * @LastEditTime: 2021-10-16 23:54:07
 * @LastEditors: Please set LastEditors
 * @Description: 机器人关节驱动头文件
 * @FilePath: /robot_software/robot/include/joint_driver.h
 */
/*
指南:
      0. 创建对象时自动运行: joint_driver();
      1. 初始化spi硬件: init_spi();
      2. 将指令存入变量: leg_cmd.
      3. 执行数据收发: update_data();
      4. 从变量中获取关节状态: leg_state, torque
*/
/*
数据的数组下标与机器人腿的对应关系:
      [0]: 右前腿
      [1]: 左前腿
      [2]: 右后腿
      [3]: 左后腿
*/

#ifndef UP_JOINT_DRIVER_H
#define UP_JOINT_DRIVER_H

#include "up_spi.h"

#include <stdio.h>//包含命名空间"std::", 和"printf" 函数
#include <stdint.h>//包含"uint8_t" 定义

/*
机器人所有腿的所有关节的指令数据结构体类型
*/
typedef struct 
{
  float q_des_abad[4]; //abad关节期望位置
  float q_des_hip[4];  //髋关节期望位置
  float q_des_knee[4]; //膝关节期望位置
  float qd_des_abad[4];//abad关节期望速度
  float qd_des_hip[4]; //髋关节期望速度
  float qd_des_knee[4];//膝关节期望速度
  float kp_abad[4];    //abad关节位置增益系数
  float kp_hip[4];     //髋关节位置增益系数
  float kp_knee[4];    //膝关节位置增益系数
  float kd_abad[4];    //abad关节速度增益系数
  float kd_hip[4];     //髋关节速度增益系数
  float kd_knee[4];    //膝关节速度增益系数
  float tau_abad_ff[4];//abad关节前馈力矩
  float tau_hip_ff[4]; //髋关节前馈力矩
  float tau_knee_ff[4];//膝关节前馈力矩
  int32_t flags[4];    //腿使能标志，0：断电，1：上电
  
}joint_command_data;

/*
机器人所有腿的所有关节的状态数据结构体类型
*/
typedef struct 
{
  float q_abad[4]; //abad关节的位置
  float q_hip[4];  //髋关节的位置
  float q_knee[4]; //膝关节的位置
  float qd_abad[4];//abad关节的速度
  float qd_hip[4]; //髋关节的速度
  float qd_knee[4];//膝关节的速度
  float qdd_abad[4];//abad关节的加速度
  float qdd_hip[4]; //髋关节的加速度
  float qdd_knee[4];//膝关节的加速度
  int32_t flags[4];//腿使能标志，0：断电，1：上电
  
}joint_state_data;

/*
机器人所有腿的所有关节的估计力矩数据结构体类型
根据当前发送的指令和返回的状态估计的关节驱动力矩
*/
typedef struct 
{
  float tau_abad[4];//abad关节驱动力矩
  float tau_hip[4]; //髋关节驱动力矩
  float tau_knee[4];//膝关节驱动力拒
  
  //伪力矩
  float fake_tau_abad[4];//abad关节驱动力矩
  float fake_tau_hip[4]; //髋关节驱动力矩
  float fake_tau_knee[4];//膝关节驱动力拒
  
}joint_torque_estimate;

/*
SPI通信一次发送的指令数据包结构体类型
各成员含义与指令数据结构体成员相同
数据大小：
        16*2+1=33 32bits
        33*2=66   16bits
        66*2=132  8bits
*/
typedef struct 
{
  float q_des_abad[2];
  float q_des_hip[2];
  float q_des_knee[2];
  float qd_des_abad[2];
  float qd_des_hip[2];
  float qd_des_knee[2];
  float kp_abad[2];
  float kp_hip[2];
  float kp_knee[2];
  float kd_abad[2];
  float kd_hip[2];
  float kd_knee[2];
  float tau_abad_ff[2];
  float tau_hip_ff[2];
  float tau_knee_ff[2];
  int32_t flags[2];
  int32_t checksum;//发送数据的32bits亦或校验
  
}send_data;

/*
SPI通信一次接收的关节状态数据结构体类型
各成员含义与关节数据结构体成员相同
数据大小：
        7*2+1=15 32bits
        15*2=30  16bits
        30*2=60  8bits
*/
typedef struct 
{
  float q_abad[2];
  float q_hip[2];
  float q_knee[2];
  float qd_abad[2];
  float qd_hip[2];
  float qd_knee[2];
  int32_t flags[2];
  int32_t checksum;//接收数据的32bits亦或校验
  
}receive_data;

/*
该类执行机器人各关节的控制，通过两个SPI通信
SPI1控制[0][1]腿，SPI2控制[2][3]腿
*/
class joint_driver
{
  public:
    joint_driver(void);

    uint8_t init_spi(void);
    uint8_t update_data(void);

    //变量：存储关节指令数据
    joint_command_data cmd;
    //变量：存储关节状态数据
    joint_state_data state;
    //变量：存储关节的估计力矩数据
    joint_torque_estimate torque;

  private:
    void     command2spi(void);
    uint8_t  spi2state(void);
    void     acc_estimate(void);
    void     torque_estimate(void);
    uint32_t xor_checksum(uint32_t *data, size_t len);

    //对象，spi1和spi2的驱动
    up_spi spi1, spi2;
    //变量：存储spi1和spi2的发送数据
    send_data tx_data1, tx_data2;
    //变量：存储spi1和spi2的接收数据
    receive_data rx_data1, rx_data2;
};

#endif //#ifndef UP_JOINT_DRIVER_H
