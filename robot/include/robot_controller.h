/*
 * @Author: sun
 * @Date: 2021-01-11 01:21:25
 * @LastEditTime: 2025-09-09 16:59:36
 * @LastEditors: Please set LastEditors
 * @Description: 机器人控制器的基类头文件
 * @FilePath: /robot_software/robot/include/robot_controller.h
 */
/*
指南: 该头文件声明的类为机器人控制器的基类
     其包含的数据由robot_console提供
     后面所有的控制器都应是该类的派生类
*/
#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "up_usbDevice.h"   //传感器采集板(USB)驱动
#include "up_at9s.h"        //遥控器驱动
#include "leg_driver.h"     //腿驱动
#include "body_state_estimator.h"  //机身状态估计

#include <stdio.h> //包含"std::"和"printf" 函数
#include <stdint.h>//包含"uint8_t" 定义

/*
机器人控制器基类，包括需要与robot_console交互的函数和数据
*/
class robot_controller
{
  friend class robot_console;
  public:
    robot_controller(void){}//构造函数，无操作
    ~robot_controller(){}   //析构函数，无操作

    virtual void init(void) = 0;//用于初始化
    virtual void run_controller(void) = 0;//用于执行控制任务
    virtual void run_print_state(void) = 0;//用于输出状态信息

  protected://以下变量由友元类robot_console赋值
    //腿驱动对象，驱动腿并存储指令和状态数据
    leg_driver* leg_drv;

    //USB设备数据，存储传感器采集板数据
    usb_device_data usb_data; 
    //遥控器指令变量，存储遥控器指令
    at9s_cmd rc_cmd;
    //变量，存储估计的身体状态结果
    body_state_estimator_data* body_data;
};

#endif //#ifndef ROBOT_CONTROLLER_H
