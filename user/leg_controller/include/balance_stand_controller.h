/*
 * @Author: sun
 * @Date: 2021-04-11 04:44:22
 * @LastEditTime: 2021-04-13 19:18:55
 * @LastEditors: Please set LastEditors
 * @Description: 机器人站立模式的平衡控制器头文件
 * @FilePath: /robot_software/user/leg_controller/include/balance_stand_controller.h
 */

/*
指南:
      0. 创建对象时自动运行: balance_stand_controller;
      1. 每次重新进入该模式时，第一个控制周期运行
         初始化函数一次: init();
      2. 每个控制周期运行控制器: run();
*/
#ifndef BALANCE_STAND_CONTROLLER_H
#define BALANCE_STAND_CONTROLLER_H

#include "up_at9s.h"             //遥控器驱动
#include "leg_driver.h"          //腿驱动
#include "body_state_estimator.h"//机身状态估计

#include "cppTypes.h" //包含Vec3<float>等与Eigen有关类型的定义
#include <stdint.h>   //包含"uint8_t"定义

class balance_stand_controller
{
  public:
    //在类中定义Eigen变量时要加入下面这句用于数据对齐
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    balance_stand_controller(at9s_cmd* _rc_cmd, leg_driver* _leg_drv, body_state_estimator_data* _body_data);

    void init(void);
    void run(void);

  private:

    //遥控器指令变量，存储遥控器指令
    at9s_cmd* rc_cmd;
    //腿驱动对象，驱动腿并存储指令和状态数据
    leg_driver* leg_drv;
    //变量，存储估计的身体状态结果
    body_state_estimator_data* body_data;

    //初始足端相对于机身的位置在机身坐标系下的表示
    Vec3<float> pb0_bf[4];

    float roll_des;  //期望滚转角
    float pitch_des; //期望俯仰角
    float yaw_des;   //期望偏航角
    float height_des;//期望机身高度

    float roll_err_old; //roll上次偏差 
    float roll_err_I;   //roll偏差积分控制量
    float pitch_err_old;//pitch上次偏差
    float pitch_err_I;  //pitch偏差积分控制量
};

#endif //#ifndef BALANCE_STAND_CONTROLLER_H
