/*
 * @Author: sun
 * @Date: 2021-04-13 23:13:47
 * @LastEditTime: 2025-01-05 23:32:00
 * @LastEditors: Please set LastEditors
 * @Description: 机器人运动模式控制器头文件
 * @FilePath: /robot_software/user/leg_controller/include/locomotion_controller.h
 */

/*
指南:
      0. 创建对象时自动运行: locomotion_controller;
      1. 每次重新进入该模式时，第一个控制周期运行
         初始化函数一次: init();
      2. 每个控制周期运行控制器: run();
*/
#ifndef LOCOMOTION_CONTROLLER_H
#define LOCOMOTION_CONTROLLER_H

#include "up_at9s.h"             //遥控器驱动
#include "leg_driver.h"          //腿驱动
#include "body_state_estimator.h"//机身状态估计
#include "up_usbDevice.h"   //传感器采集板(USB)驱动
#include "fb_dynamic.h"     //动力学计算
#include "gait_plan.h"      //步态规划
#include "trajectory_plan.h"//轨迹规划

#include "WBC_Ctrl.h"
#include "LocomotionCtrl.h"


#include "cppTypes.h" //包含Vec3<float>等与Eigen有关类型的定义
#include <stdint.h>   //包含"uint8_t"定义

class locomotion_controller
{
  public:
    //在类中定义Eigen变量时要加入下面这句用于数据对齐
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    locomotion_controller(at9s_cmd* _rc_cmd, leg_driver* _leg_drv, body_state_estimator_data* _body_data, fb_dynamic* _dynamic, usb_device_data* _usb_data);

    void init(void);
    void run(void);

    void run_MPC(void);

    //步态规划对象
    gait_plan trot_gait;

    float vx_des;  //期望前后方向速度，前+
    float vy_des;  //期望左右方向速度，左+
    float vyaw_des;//期望偏航角速度，逆+
    float hf_des;  //期望抬腿高度

    float body_height;//机身目标高度
    
    bool contact_flag[4];//实际足端接触状态,从leg_controller中的状态估计器中,通过动力学计算足端力判断是否落地

    Vec3<float> pb_hf0[4];//摆动轨迹起点
    Vec3<float> pb_hf1[4];//摆动轨迹终点
    Vec3<float> pb_hfc[4];//摆动轨迹当前位置点
    Vec3<float> f_ff[4];//mpc计算的各腿足端力
    Vec3<float> tau_ff[4];//mpc计算的各腿足端力对应的关节力

    double mpc_real_dt;//记录MPC运行一次的真实时间

  private:

    //遥控器指令变量，存储遥控器指令
    at9s_cmd* rc_cmd;
    //腿驱动对象，驱动腿并存储指令和状态数据
    leg_driver* leg_drv;
    //变量，存储估计的身体状态结果
    body_state_estimator_data* body_data;
    //USB设备数据指针，指针存储传感器采集板数据的对象
    usb_device_data* usb_data;
    
    fb_dynamic* dynamic;

    //各腿的轨迹规划对象
    trajectory_plan tra[4];

    WBC_Ctrl<float>* _wbc_ctrl;
    LocomotionCtrlData<float>* _wbc_data;

    bool stance_control_flag[4];//支撑相控制标志, 1-支撑相, 0-摆动相
    
    //初始足端相对于机身的位置在机身坐标系下的表示
    Vec3<float> pb0_bf[4];

    bool first_stance[4];//当前周期第一次进入支撑相标志，true：第一次
    bool first_swing[4]; //当前周期第一次进入摆动相标志，true：第一次

    const float step_max  = 0.3;//m
    const float vx_max  = 0.6;//前进速度，m/s
    const float vy_max  = 0.4;//m/s
    const float vyaw_max = 2;//rad/s
    const float hight_max = 0.1;//m

    const float vyaw_correct = 0.f;

    float dtmpc;//一个mpc步长的时间=步态周期/预测步长
    float x_comp_integral;//x方向速度对z轴位置的影响

};

#endif //#ifndef LOCOMOTION_CONTROLLER_H
