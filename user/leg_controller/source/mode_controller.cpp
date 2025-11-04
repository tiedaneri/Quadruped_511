/*
 * @Author: sun
 * @Date: 2021-03-22 05:19:16
 * @LastEditTime: 2024-10-15 17:11:58
 * @LastEditors: Please set LastEditors
 * @Description: 控制模式源文件
 * @FilePath: /robot_software/user/leg_controller/source/mode_controller.cpp
 */

#include "mode_controller.h"
#include "robot.h"
#include <stdio.h>//包含"std::"和"printf"定义

/**
 * @description: 构造函数，初始化参数
 * @param {无}
 * @return {无}
 */
mode_controller::mode_controller(at9s_cmd* _rc_cmd, leg_driver* _leg_drv, body_state_estimator_data* _body_data, fb_dynamic* _dynamic, usb_device_data* _usb_data)
{
    rc_cmd = _rc_cmd;//连接遥控器指令
    leg_drv = _leg_drv;//连接腿驱动
    body_data = _body_data;//连接机身状态数据
    usb_data = _usb_data;//连接usb设备数据
    dynamic = _dynamic;//连接浮基动力学

    mode = power_off;//设置初始模式为掉电模式
    
    //模式运行时间归零
    power_on_time   = 0.f;
    fold_time       = 0.f;
    stand_up_time   = 0.f;
    stand_down_time = 0.f;

    first_run_balance = true;
    first_run_locomotion = true;
}

/**
 * @description: 析构函数，释放内存
 * @param {无}
 * @return {无}
 */
mode_controller::~mode_controller()
{
    delete balance_stand_ctr;//删除balance_stand_ctr对象
    delete locomotion_ctr;   //删除locomotion_ctr对象
    printf("[mode_controller] mode_controller ended\n");
}

/**
 * @description: 初始化参数
 * @param {无}
 * @return {无}
 */
void mode_controller::init()
{
    //创建平衡站立控制对象
    balance_stand_ctr = new balance_stand_controller(rc_cmd, leg_drv, body_data);
    //创建运动控制对象
    locomotion_ctr = new locomotion_controller(rc_cmd, leg_drv, body_data, dynamic, usb_data);
    
    //设置初始遥控器指令
    rc_cmd->SWA = UP;
    rc_cmd->SWB = UP;
    rc_cmd->SWC = UP;
    rc_cmd->SWD = UP;
    rc_cmd->SWE = DOW;
    rc_cmd->SWF = DOW;
    rc_cmd->SWG = DOW;
    rc_cmd->SWH = DOW;
    rc_cmd->LRO = 0.f;
    rc_cmd->LCO = 0.f;
    rc_cmd->RRO = 0.f;
    rc_cmd->RCO = 0.f;
    rc_cmd->VA  = 0.f;
    rc_cmd->VB  = 0.f;
}

/**
 * @description: 根据遥控器指令和当前模式状态设置控制模式
 * @param {无}
 * @return {无}
 */
void mode_controller::set_mode()
{
    //设置遥控器指令大小范围
    if(rc_cmd->LRO >  0.5) rc_cmd->LRO =  0.5;
    if(rc_cmd->LRO < -0.5) rc_cmd->LRO = -0.5;
    if(rc_cmd->LCO >  0.5) rc_cmd->LCO =  0.5;
    if(rc_cmd->LCO < -0.5) rc_cmd->LCO = -0.5;
    if(rc_cmd->RRO >  0.5) rc_cmd->RRO =  0.5;
    if(rc_cmd->RRO < -0.5) rc_cmd->RRO = -0.5;
    if(rc_cmd->RCO >  0.5) rc_cmd->RCO =  0.5;
    if(rc_cmd->RCO < -0.5) rc_cmd->RCO = -0.5;
    if(rc_cmd->VB  <  0)   rc_cmd->VB  =  0;
    if(rc_cmd->VB  >  1)   rc_cmd->VB  =  1;

    if(rc_cmd->SWE == DOW)//power_off状态
    {
        //只有power_on模式可以转换到该模式
        if(mode == power_on && power_on_time > power_on_time_max)
        {
            mode = power_off;   //设置为power_off模式
            power_on_time = 0.f;//上一个模式时间清零
            printf("[mode set] {power_on} to {power_off}\n");
        }
        //其他模式则先转换成power_on模式
        if(mode != power_on && mode != power_off) 
        {
            mode = power_on;
            //其他模式时间清零
            fold_time = 0.f;
            stand_up_time = 0.f;
            stand_down_time = 0.f;
            first_run_balance = true;
            first_run_locomotion = true;
            printf("[mode set] {other mode} to {power_on}\n");
        }
        //power_off模式下无操作，保持原模式
    }

    if(rc_cmd->SWE == MID)//power_on状态
    {
        //任何模式都可以直接转换成该模式
        if(mode != power_on)
        {
            mode = power_on;
            //其他模式时间清零
            fold_time = 0.f;
            stand_up_time = 0.f;
            stand_down_time = 0.f;
            first_run_balance = true;
            first_run_locomotion = true;
            printf("[mode set] {other mode} to {power_on}\n");
        }
        //power_on模式下无操作，保持原模式
    }

    if(rc_cmd->SWE == UP && rc_cmd->SWG == DOW)//fold状态
    {
        //power_on模式下进入fold模式
        if(mode == power_on && power_on_time > power_on_time_max)
        {
            mode = fold;
            power_on_time = 0.f;
            printf("[mode set] {power_on} to {fold}\n");
        }
        //stand_up模式下进入stand_down模式
        if(mode == stand_up && stand_up_time > stand_up_time_max)
        {
            mode = stand_down;
            stand_up_time = 0.f;
            printf("[mode set] {stand_up} to {stand_down}\n");
        }
        //balance模式和locomotion模式下进入stand_down模式
        if(mode == balance || mode == locomotion) 
        {
            mode = stand_down;
            first_run_balance = true;
            first_run_locomotion = true;
            printf("[mode set] {balance/locomotion} to {stand_down}\n");
        }
        //estop、power_off、stand_down、fold模式下无操作，保持原模式
    }

    if(rc_cmd->SWE == UP && rc_cmd->SWG == MID)//stand_up状态
    {
        //power_on模式下进入fold模式
        if(mode == power_on && power_on_time > power_on_time_max)
        {
            mode = fold;
            power_on_time = 0.f;
            printf("[mode set] {power_on} to {fold}\n");
        }
        //fold模式下进入stand_up模式
        if(mode == fold && fold_time > fold_time_max)
        {
            mode = stand_up;
            fold_time = 0.f;
            printf("[mode set] {fold} to {stand_up}\n");
        }
        //stand_down模式下进入stand_up模式
        if(mode == stand_down && stand_down_time > stand_down_time_max)
        {
            mode = stand_up;
            stand_down_time = 0.f;
            printf("[mode set] {stand_down} to {stand_up}\n");
        }
        //balance模式和locomotion模式下进入stand_up模式
        if(mode == balance || mode == locomotion) 
        {
            mode = stand_up;
            first_run_balance = true;
            first_run_locomotion = true;
            printf("[mode set] {balance/locomotion} to {stand_up}\n");
        }
        //estop、power_off、stand_up模式下无操作，保持原模式
    }

    if(rc_cmd->SWE == UP && rc_cmd->SWG == UP && rc_cmd->SWA == UP)//balance状态
    {
        //power_on模式下进入fold模式
        if(mode == power_on && power_on_time > power_on_time_max)
        {
            mode = fold;
            power_on_time = 0.f;
            printf("[mode set] {power_on} to {fold}\n");
        }
        //fold模式下进入stand_up模式
        if(mode == fold && fold_time > fold_time_max)
        {
            mode = stand_up;
            fold_time = 0.f;
            printf("[mode set] {fold} to {stand_up}\n");
        }
        //stand_up模式下进入balance模式
        if(mode == stand_up && stand_up_time > stand_up_time_max)
        {
            mode = balance;
            stand_up_time = 0.f;
            printf("[mode set] {stand_down} to {balance}\n");
        }
        //stand_down模式下进入stand_up模式
        if(mode == stand_down && stand_down_time > stand_down_time_max)
        {
            mode = stand_up;
            stand_down_time = 0.f;
            printf("[mode set] {stand_down} to {stand_up}\n");
        }
        //locomotion模式下进入balance模式
        if(mode == locomotion)
        {
            mode = balance;
            first_run_locomotion = true;
            printf("[mode set] {locomotion} to {balance}\n");
        }
        //estop、power_off、balance模式下无操作，保持原模式
    }

    if(rc_cmd->SWE == UP && rc_cmd->SWG == UP && rc_cmd->SWA == DOW)//locomotion状态
    {
        //power_on模式下进入fold模式
        if(mode == power_on && power_on_time > power_on_time_max)
        {
            mode = fold;
            power_on_time = 0.f;
            printf("[mode set] {power_on} to {fold}\n");
        }
        //fold模式下进入stand_up模式
        if(mode == fold && fold_time > fold_time_max)
        {
            mode = stand_up;
            fold_time = 0.f;
            printf("[mode set] {fold} to {stand_up}\n");
        }
        //stand_up模式下进入locomotion模式
        if(mode == stand_up && stand_up_time > stand_up_time_max)
        {
            mode = locomotion;
            stand_up_time = 0.f;
            printf("[mode set] {stand_up} to {locomotion}\n");
        }
        //stand_down模式下进入stand_up模式
        if(mode == stand_down && stand_down_time > stand_down_time_max)
        {
            mode = stand_up;
            stand_down_time = 0.f;
            printf("[mode set] {stand_down} to {stand_up}\n");
        }
        //balance模式下进入locomotion模式
        if(mode == balance)
        {
            mode = locomotion;
            first_run_balance = true;
            printf("[mode set] {balance} to {locomotion}\n");
        }
        //estop、power_off、locomotion模式下无操作，保持原模式
    }
}

/**
 * @description: 运行当前设定的模式
 * @param {无}
 * @return {无}
 */
void mode_controller::run_mode()
{
    if(mode == estop)      estop_mode();
    if(mode == power_off)  power_off_mode();
    if(mode == power_on)   power_on_mode();
    if(mode == fold)       fold_mode();
    if(mode == stand_up)   stand_up_mode();
    if(mode == stand_down) stand_down_mode();
    if(mode == balance)    balance_mode();
    if(mode == locomotion) locomotion_mode();
}

/**
 * @description: estop_mode功能函数
 * @param {无}
 * @return {无}
 */
void mode_controller::estop_mode()
{
    //上电无输出
    leg_drv->zero_cmd();
    leg_drv->leg_enable = true;//power on
}

/**
 * @description: power_off_mode功能函数
 * @param {无}
 * @return {无}
 */
void mode_controller::power_off_mode()
{
    //断电无输出
    leg_drv->zero_cmd();
    leg_drv->leg_enable = false;//power off
}

/**
 * @description: power_on_mode 功能函数
 * @param {无}
 * @return {无}
 */
void mode_controller::power_on_mode()
{
    //上电无输出
    leg_drv->leg_enable = true;
    //模式进入时间计时
    if(power_on_time <= power_on_time_max) power_on_time += robot::dt;
    power_on_time_pub = power_on_time;
}

/**
 * @description: fold_mode 功能函数
 * @param {无}
 * @return {无}
 */
void mode_controller::fold_mode()
{
    leg_drv->leg_enable = true;//上电
    for(int leg = 0; leg < 4; leg++)//启用电机PD输出
    {
        leg_drv->cmd[leg].kp = Vec3<float>(60, 60, 60).asDiagonal() * ROBOT_K;
        leg_drv->cmd[leg].kd = Vec3<float>(1.2, 1.2, 1.2).asDiagonal() * ROBOT_K;
    }
    if(fold_time == 0.f)//第一次运行，记录当前位置作为初始位置
    {
        for(int leg = 0; leg < 4; leg++) initial_jpos[leg] = leg_drv->state[leg].q;
    }
    if(fold_time <= fold_time_max)//运动到目标位置
    {
        float rat = fold_time/fold_time_max;
        for(int leg = 0; leg < 4; leg++)
            leg_drv->cmd[leg].q_des = rat * robot::fold_jpos + (1-rat) * initial_jpos[leg];
        fold_time += robot::dt;
    }
    else//保持目标位置
    {
        for(int leg = 0; leg < 4; leg++)
        leg_drv->cmd[leg].q_des = robot::fold_jpos;
    } 
}

/**
 * @description: stand_up_mode 功能函数
 * @param {无}
 * @return {无}
 */
void mode_controller::stand_up_mode()
{
    leg_drv->leg_enable = true;//上电
    for(int leg = 0; leg < 4; leg++)//启用电机PD输出
    {
        leg_drv->cmd[leg].kp = Vec3<float>(60, 60, 60).asDiagonal() * ROBOT_K;
        leg_drv->cmd[leg].kd = Vec3<float>(1.2, 1.2, 1.2).asDiagonal() * ROBOT_K;
    }
    if(stand_up_time == 0.f)//第一次运行，记录当前位置作为初始位置
    {
        for(int leg = 0; leg < 4; leg++) initial_jpos[leg] = leg_drv->state[leg].q;
    }
    if(stand_up_time <= stand_up_time_max)//运动到目标位置
    {
        float rat = stand_up_time/stand_up_time_max;
        for(int leg = 0; leg < 4; leg++)
            leg_drv->cmd[leg].q_des = rat * robot::stand_jpos + (1-rat) * initial_jpos[leg];
        stand_up_time += robot::dt;
    }
    else//保持目标位置
    {
        for(int leg = 0; leg < 4; leg++)
         leg_drv->cmd[leg].q_des = robot::stand_jpos;
    } 
}

/**
 * @description: stand_down_mode 功能函数
 * @param {无}
 * @return {无}
 */
void mode_controller::stand_down_mode()
{
    leg_drv->leg_enable = true;//上电
    for(int leg = 0; leg < 4; leg++)//启用电机PD输出
    {
        leg_drv->cmd[leg].kp = Vec3<float>(60, 60, 60).asDiagonal() * ROBOT_K;
        leg_drv->cmd[leg].kd = Vec3<float>(1.2, 1.2, 1.2).asDiagonal() * ROBOT_K;
    }
    if(stand_down_time == 0.f)//第一次运行，记录当前位置作为初始位置
    {
        for(int leg = 0; leg < 4; leg++) initial_jpos[leg] = leg_drv->state[leg].q;
    }
    if(stand_down_time <= stand_down_time_max)//运动到目标位置
    {
        float rat = stand_down_time/stand_down_time_max;
        for(int leg = 0; leg < 4; leg++)
            leg_drv->cmd[leg].q_des = rat * robot::fold_jpos + (1-rat) * initial_jpos[leg];
        stand_down_time += robot::dt;
    }
    else//保持目标位置
    {
        for(int leg = 0; leg < 4; leg++)
            leg_drv->cmd[leg].q_des = robot::fold_jpos;
    } 
}

/**
 * @description: balance_mode 功能函数
 * @param {无}
 * @return {无}
 */
void mode_controller::balance_mode()
{
    leg_drv->leg_enable = true;//上电
    for(int leg = 0; leg < 4; leg++)//启用电机PD输出
    {
        leg_drv->cmd[leg].kp = Vec3<float>(60, 60, 60).asDiagonal() * ROBOT_K;
        leg_drv->cmd[leg].kd = Vec3<float>(1.2, 1.2, 1.2).asDiagonal() * ROBOT_K;
    }
    
    if(first_run_balance)//每次重新进入该模式时都要初始化
    {
        balance_stand_ctr->init();
        first_run_balance = false;
    }
    balance_stand_ctr->run();
}

/**
 * @description: locomotion_mode 功能函数
 * @param {无}
 * @return {无}
 */
void mode_controller::locomotion_mode()
{
    leg_drv->leg_enable = true;//上电
    // for(int leg = 0; leg < 4; leg++)//启用电机PD输出
    // {
    //     leg_drv->cmd[leg].kp = Vec3<float>(80, 80, 80).asDiagonal();
    //     leg_drv->cmd[leg].kd = Vec3<float>(2, 2, 2).asDiagonal();
    // }

    if(first_run_locomotion)//每次重新进入该模式时都要初始化
    {
        locomotion_ctr->init();
        first_run_locomotion = false;
    }
    locomotion_ctr->run();
    
}

