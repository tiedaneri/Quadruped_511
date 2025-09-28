/*
 * @Author: sun
 * @Date: 2021-04-14 17:51:25
 * @LastEditTime: 2024-09-24 10:41:11
 * @LastEditors: Please set LastEditors
 * @Description: 机器人步态规划头文件
 * @FilePath: /robot_software/user/leg_controller/include/gait_plan.h
 */

/*
指南:
      0. 创建对象时自动运行: gait_plan;
      1. 每次进入该步态执行一次：init
      1. 设置步态参数: T，c_gait，phase_offset，phase_stance;
      2. 生效步态参数：set();
      3. 每个控制周期运行一次更新实时步态状态: run();
      4. 获取步态参数和状态
      5. 注：每个周期中，先是支撑相，后是摆动相
*/

#ifndef GAIT_PLAN_H
#define GAIT_PLAN_H

#include <stdint.h>//包含"uint8_t"定义

class gait_plan
{
  public:
    gait_plan(void);

    void init(void);
    void set(void);
    void run(void);
    void run_mpc_stance_flag(float mpc_t);
    
///////////////////////////////设定的步态参数

    //变量，存储一个dt_gait，控制器执行的次数
    int c_gait;
    //变量，存储一个完整步态的时间,s
    float T;
    //变量，存储各腿的相位偏置
    float phase_offset[4];
    //变量，存储各腿支撑相的占比
    float phase_stance[4];

//////////////////////////////实时更新的参数

    //变量，存储控制器实时次数
    float c;
    //变量，存储实时时间，s(0~T)
    float t;
    
//////////////////////////////步态参数

    //变量，存储步态控制的最小时间单位（多久执行一次run()）,s
    float dt_gait;
    //变量，存储支撑相总时间，s
    float T_stance[4];
    //变量，存储摆动相总时间，s
    float T_swing[4];

//////////////////////////////实时变化的步态状态

    //变量，存储各腿是否处于支撑相
    //true,1:支撑相，false,0:摆动相
    bool stance_flag[4];
    //变量， 存储支撑状态，支撑时间变量占支撑相时间的比值, 0-1
    float stance_state[4];
    //变量， 存储摆动状态，摆动时间变量占摆动相时间的比值, 0-1
    float swing_state[4];
    //变量，存储进入支撑相的实时时间，s
    float t_stance[4];
    //变量，存储进入摆动相的实时时间，s
    float t_swing[4];

    //变量，存储mpc每条腿支撑状态
    int mpc_stance_flag[4];

  private:
    //变量，存储各腿的相位偏置时间
    float T_offset[4];

};

#endif //GAIT_PLAN_H
