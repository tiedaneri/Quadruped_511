/*
 * @Author: sun
 * @Date: 2021-04-14 17:50:31
 * @LastEditTime: 2024-09-24 10:09:31
 * @LastEditors: Please set LastEditors
 * @Description: 机器人步态规划源文件
 * @FilePath: /robot_software/user/leg_controller/source/gait_plan.cpp
 */

#include "gait_plan.h"
#include "robot.h"

#include <stdio.h>//包含"std::"和"printf"定义
#include <math.h>

/**
 * @description: 构造函数，初始化参数
 * @param {无}
 * @return {无}
 */
gait_plan::gait_plan()
{
    init();
}

/**
 * @description: 设置各变量初始值,每次重新进入该步态时要执行一次
 * @param {无}
 * @return {无}
 */
void gait_plan::init()
{
//////////////////////////////设置初始步态参数

    c_gait = 9; //经过这些次控制周期(0.002s)后,运行一次gait_plan
    T = 0.6; //一条腿运行完一次摆动相和支撑相的时间, 在locomotion_controller中设置为0.4s
    
    //对角步态,四条腿的偏置时间与步态周期T的比值,对应论文中P27 k_offset
    phase_offset[0] = 0;
    phase_offset[1] = 0.5; 
    phase_offset[2] = 0.5;
    phase_offset[3] = 0;

    //一半占空比, 四条腿的支撑相与步态周期的比值, 对应论文中P27 k_st
    phase_stance[0] = 0.5;
    phase_stance[1] = 0.5;
    phase_stance[2] = 0.5;
    phase_stance[3] = 0.5;

    for(int leg = 0; leg < 4; leg++)
    {
        stance_flag[leg] = true;
        stance_state[leg] = 1;
        swing_state[leg] = 0;
    }
    
    c = 0;
    t = 0;
    dt_gait = 0.02;//随便设置的值，set()后会变
    set();
}

/**
 * @description: 配置步态的参数后运行该函数，使配置生效
 * @param {无}
 * @return {无}
 */
void gait_plan::set()
{
    // dt_gait = c_gait * robot::dt;//不使用MPC
    dt_gait = c_gait * robot::dt + robot::mpc_time;//使用MPC,0.023s
    
    for(int leg = 0; leg < 4; leg++)
    {
        T_stance[leg] = T * phase_stance[leg];
        T_swing[leg]  = T * (1 - phase_stance[leg]);
        T_offset[leg] = T * phase_offset[leg];
    }
}

/**
 * @description: 每10个控制周期运行一次，更新实时时间、是否摆动相、各腿支撑相和摆动相的实时时间
 * @param {无}
 * @return {无}
 */
void gait_plan::run()
{
    c++;//控制器实时次数加1
    if(c > c_gait)//一个步态控制的最小时间单位
    {
        c = 0;
        t += dt_gait;//更新实时时间
        if(t >= T) t = 0;//一整个步态周期后重载

        for(int leg = 0; leg < 4; leg++)
        {   
            float t_offset;//偏置后所在周期的真实时间

            //计算所在周期的偏置时间,对应论文P28上半
            if(t < T_offset[leg])//在下个周期
            {
                t_offset = t + T - T_offset[leg];//转换到所在周期的真实时间
            }
            else//t >= T_offset[leg],在当前周期
            {
                t_offset = t - T_offset[leg];//转换到所在周期的真实时间
            }
            
            //计算支撑/摆动的相位,对应论文P28下半
            if(t_offset < T_stance[leg]) //处于当前周期的支撑相
            {
                stance_flag[leg] = true; //更新支撑相状态
                t_stance[leg] = t_offset;//更新当前周期进入支撑相的实时时间
                //更新支撑状态, 支撑时间变量占支撑相时间的比值
                stance_state[leg] = t_stance[leg] / T_stance[leg];
                //更新摆动状态, 摆动时间变量占摆动相时间的比值
                swing_state[leg] = 0;
            }
            else//t_offset >= T_stance[leg],处于当前周期的摆动相
            {
                stance_flag[leg] = false;//更新支撑相状态
                t_swing[leg] = t_offset - T_stance[leg];//更新当前周期进入摆动相的实时时间
                //更新支撑状态
                stance_state[leg] = 0;
                //更新摆动状态
                swing_state[leg] = t_swing[leg] / T_swing[leg];
            }
        }
    }
}

/**
 * @description: 预测每个mpc步长中的每条腿支撑状态
 * @param {float mpc_t} 全局周期的时间变量
 * @return {无}
 */
void gait_plan::run_mpc_stance_flag(float mpc_t)
{
    float pt = mpc_t;
    while(pt >= T) pt = pt - T;//将时间变量变小到一个周期里
    for(int leg = 0; leg < 4; leg++)
    {   
        float t_offset;//偏置后所在周期的真实时间

        //计算所在周期的偏置时间
        if(pt < T_offset[leg])//在下个周期
        {
            t_offset = pt + T - T_offset[leg];//转换到所在周期的真实时间
        }
        else//t >= T_offset[leg],在当前周期
        {
            t_offset = pt - T_offset[leg];//转换到所在周期的真实时间
        }
        
        //计算支撑相状态
        if(t_offset < T_stance[leg]) //处于当前周期的支撑相
        {
            mpc_stance_flag[leg] = 1; //更新支撑相状态
        }
        else//t_offset >= T_stance[leg],处于当前周期的摆动相
        {
            mpc_stance_flag[leg] = 0;//更新支撑相状态
        }
    }
}