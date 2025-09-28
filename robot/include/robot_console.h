/*
 * @Author: sun
 * @Date: 2021-01-03 22:52:12
 * @LastEditTime: 2025-09-09 16:20:10
 * @LastEditors: Please set LastEditors
 * @Description: 机器人控制台头文件
 *               初始化硬件配置，管理数据，管理和启动控制任务
 * @FilePath: /robot_software/robot/include/robot_console.h
 */
/*
指南:
      0. 创建对象时自动运行: robot_console();
      1. 初始化硬件和软件: init_sum();
      2. 创建和启动任务: run_tasks();
*/
#ifndef ROBOT_CONSOLE_H
#define ROBOT_CONSOLE_H

#include "up_usbDevice.h"          //传感器采集板(USB)驱动
#include "up_at9s.h"               //遥控器驱动
#include "joint_driver.h"          //关节驱动
#include "lord_imu/LordImu.h"      //陀螺仪驱动

#include "leg_driver.h"            //腿驱动
#include "body_state_estimator.h"  //机身状态估计
#include "robot_controller.h"      //机器人控制器

#include "Utilities/PeriodicTask.h"//任务管理

#include <stdio.h> //包含"std::"和"printf" 函数
#include <stdint.h>//包含"uint8_t" 定义
#include <mutex>   //互斥线程锁

/*
机器人控制台类，初始化硬件配置，管理数据，管理和启动控制任务
*/
class robot_console
{
  public:
    robot_console(robot_controller* rt_ctr);
    ~robot_console();

    uint8_t init_sum(void);
    void run_tasks(void);
    
    //变量，程序终止标志，0：运行，1：终止，
    //该值在main.cpp中修改，用于终止程序
    uint8_t stop_cmd;

  private:
    uint8_t prefaultStack(void);
    uint8_t setupScheduler(void);
    uint8_t init_hardware(void);
    void init_controller(void);
    void print_joint_cmd(void);
    void print_joint_state(void);

    void run_usb_device(void); 
    void run_remote(void);
    void run_imu(void);
    void run_joint_drive(void);
    void run_robot_controller(void);
    void run_print_state(void);

    //关节状态数据线程锁
    std::mutex jdata_m;
    //关节指令数据线程锁
    std::mutex jcmd_m;
    //USB设备数据线程锁
    std::mutex usb_data_m;
    //遥控器指令数据线程锁
    std::mutex rcmd_m;
    //IMU数据线程锁
    std::mutex imu_m;

    //传感器采集板(USB)驱动对象
    usb_device usb_dev;
    //遥控器驱动对象
    up_at9s at9s;
    //关节驱动对象
    joint_driver joint_drv;
    //imu驱动对象
    LordImu imu;

    //腿驱动对象
    leg_driver leg_drv;
    //机身状态估计对象
    body_state_estimator body_se;
    //机器人控制器对象
    robot_controller* robot_ctr;

    //任务管理对象
    PeriodicTaskManager taskmanager;

    //关节指令数据，用于线程间数据交换
    joint_command_data joint_cmd;
    //关节状态数据，用于线程间数据交换
    joint_state_data joint_state;
    //关节状态数据，用于线程间数据交换
    joint_torque_estimate joint_torque;
    
    //USB设备数据，用于线程间交换
    usb_device_data usb_data;     
    //遥控器指令数据，用于线程间数据交换
    at9s_cmd rc_cmd;

    //IMU数据，用于线程间数据交换
    float quat[4];
    float gyro[3];
    float  acc[3];

};

#endif
