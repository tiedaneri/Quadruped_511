/*
 * @Author: sun
 * @Date: 2021-01-18 09:04:28
 * @LastEditTime: 2024-09-19 15:40:15
 * @LastEditors: Please set LastEditors
 * @Description: 腿驱动头文件
 *               使用Eigen库，支持矩阵运算，
 *               面向上层控制器，连接下层关节驱动
 * @FilePath: /robot_software/robot/include/leg_driver.h
 */

/*
指南:
      0. 创建对象时自动运行: leg_driver();
      1. 在控制器运行之前更新状态: update_state();
         并将控制指令清空
      2. 运行控制器程序
      3. 控制器程序结束后，将指令更新到关节驱动: update_cmd();
*/

#ifndef LEG_DRIVER_H
#define LEG_DRIVER_H

#include "joint_driver.h"//包含关节驱动类
#include "robot.h"//包含机器人参数和运动学函数
//包含Vec3<float>等与Eigen有关类型的定义
#include "cppTypes.h"

//结构体（类）类型，腿的控制指令
struct leg_command_data 
{
  //在类中定义Eigen变量时要加入下面这句用于数据对齐
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  //构造函数，所有数据清零
  leg_command_data() { zero(); }
  void zero();
  //abad关节、髋关节、膝关节的：
  //期望位置、期望速度、前馈力矩
  //比例控制系数、微分控制系数
  Vec3<float> q_des, qd_des, tau_ff;
  Mat3<float> kp, kd;//对角阵
};

//结构体（类）类型，单腿的状态数据
struct leg_state_data
{
  //在类中定义Eigen变量时要加入下面这句用于数据对齐
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  //构造函数，所有数据清零
  leg_state_data() { zero(); }
  void zero();

  //abad关节、髋关节、膝关节的：
  //位置、速度、加速度、力矩
  Vec3<float> q, qd, qdd, tau;

  //伪力矩
  Vec3<float> fake_tau;
  
  //该腿基坐标系到足端坐标系的雅克比矩阵
  Mat3<float> J;
  
  //足端在腿基坐标系下x轴方向、y轴方向、z轴方向的：位置、速度
  Vec3<float> p, v;
  
  //位置、速度、关节向足端投影的力
  // Vec3<float> p, v, f;
};

/*
腿驱动类
使用Eigen库，支持矩阵运算，
面向上层控制器，连接下层关节驱动
*/
class leg_driver 
{
  //robot_console中需要访问和修改底层通信数据
  friend class robot_console;
  public:
    leg_driver(void);

    void zero_cmd(void);

    //变量，存储4条腿的控制指令
    //右前腿、左前腿、右后腿、左后腿
    leg_command_data cmd[4];
    //变量，存储4条腿的状态信息
    //右前腿、左前腿、右后腿、左后腿
    leg_state_data state[4];
    //变量，所有腿的使能标志，false：断电，true：上电
    bool leg_enable;

  protected:
    //状态更新和指令更新只能在robot_console中执行
    void update_state(void);
    void update_cmd(void);
    
    //变量：存储关节指令数据
    joint_command_data joint_cmd;
    //变量：存储关节状态数据
    joint_state_data joint_state;
    //变量：存储关节的估计力矩数据
    joint_torque_estimate joint_torque;
    
};

#endif // #ifndef LEG_DRIVER_H
