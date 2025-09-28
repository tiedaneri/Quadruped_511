/*
 * @Author: sun
 * @Date: 2021-03-22 05:20:03
 * @LastEditTime: 2024-09-24 17:26:23
 * @LastEditors: Please set LastEditors
 * @Description: 控制模式头文件
 * @FilePath: /robot_software/user/leg_controller/include/mode_controller.h
 */

/*
指南:
      0. 创建对象时自动运行: mode_controller;
      1. 运行初始化函数: init();
      2. 根据当前状态和指令设置模式: set_mode();
      3. 运行模式控制器: run_mode();
*/
#ifndef MODE_CONTROLLER_H
#define MODE_CONTROLLER_H

#include "up_at9s.h"        //遥控器驱动
#include "leg_driver.h"     //腿驱动
#include "body_state_estimator.h"  //机身状态估计
#include "fb_dynamic.h"     //动力学计算
#include "trajectory_plan.h"//轨迹规划
#include "balance_stand_controller.h"
#include "locomotion_controller.h"

#include <stdint.h>         //包含"uint8_t"定义

typedef enum //控制模式枚举类型
{
  estop      = 0,//出错急停模式
  power_off  = 1,//断电模式
  power_on   = 2,//上电无输出模式
  fold       = 3,//收腿模式
  stand_up   = 4,//站立模式
  stand_down = 5,//下蹲模式
  balance    = 6,//平衡模式
  locomotion = 7 //运动模式
  
}mode_type;

class mode_controller
{
  public:
    mode_controller(at9s_cmd* _rc_cmd, leg_driver* _leg_drv, body_state_estimator_data* _body_data, fb_dynamic* _dynamic);
    ~mode_controller();
    
    void init(void);
    void set_mode(void);
    void run_mode(void);
    
    //变量，存储当前操作模式
    mode_type mode;
    float power_on_time_pub;
    balance_stand_controller* balance_stand_ctr;
    locomotion_controller* locomotion_ctr;
    
  private:
    void estop_mode(void);
    void power_off_mode(void);
    void power_on_mode(void);
    void fold_mode(void);
    void stand_up_mode(void);
    void stand_down_mode(void);
    void balance_mode(void);
    void locomotion_mode(void);

    //遥控器指令变量，存储遥控器指令
    at9s_cmd* rc_cmd;
    //腿驱动对象，驱动腿并存储指令和状态数据
    leg_driver* leg_drv;
    //变量，存储估计的身体状态结果
    body_state_estimator_data* body_data;

    fb_dynamic* dynamic;
    
    //变量，记录动作的初始位置
    Vec3<float> initial_jpos[4];

    //变量，记录各模式运行时间
    float power_on_time;
    float fold_time;
    float stand_up_time;
    float stand_down_time;

    //常量，存储各模式完成所需时间, 必须为0.002的倍数
    const float power_on_time_max   = 2.f;
    const float fold_time_max       = 3.f;
    const float stand_up_time_max   = 2.f;
    const float stand_down_time_max = 2.f;

    //记录在该次进入balance_mode后是否第一次运行
    bool first_run_balance;
    //记录在该次进入locomotion_mode后是否第一次运行
    bool first_run_locomotion;
};

#endif //#ifndef MODE_CONTROLLER_H
