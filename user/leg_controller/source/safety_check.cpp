/*
 * @Author: your name
 * @Date: 2021-03-24 06:29:11
 * @LastEditTime: 2024-02-25 18:45:09
 * @LastEditors: Please set LastEditors
 * @Description: safety check源文件
 *               检查机器人状态数据和指令数据，判断是否安全
 * @FilePath: /robot_software/user/leg_controller/source/safety_check.cpp
 */

#include "safety_check.h"

#include "cppTypes.h"//包含Vec3<float>等与Eigen有关类型的定义
#include <stdio.h> //包含"std::"和"printf" 函数
#include <stdint.h>//包含"uint8_t" 定义

/**
 * @description: 构造函数，连接对象
 * @param {无}
 * @return {无}
 */
safety_check::safety_check(leg_driver* _leg_drv, at9s_cmd* _rc_cmd, mode_controller* _mode_ctr, body_state_estimator_data* _body_data)
{
    leg_drv  = _leg_drv;   //连接腿驱动对象
    rc_cmd   = _rc_cmd;    //连接遥控器驱动对象
    mode_ctr = _mode_ctr;  //连接模式控制对象
    body_data = _body_data;//连接机身状态估计数据

    state_check_code = 0;
    cmd_check_code = 0;
}

/**
 * @description:  机器人状态安全检查
 * @param {无}
 * @return {无}
 */
void safety_check::run_state_check()
{
    //任何上电模式下都进行的关节力矩检查
    //任何上电模式下都进行的机身方位状态检查
    if(mode_ctr->mode != power_off)
    {
        // torque_state_check();
        orientation_state_check();
    }
    //fold模式状态检查
    if(mode_ctr->mode == fold) fold_state_check();
    //stand_up和stand_down模式状态检查
    if(mode_ctr->mode == stand_up || mode_ctr->mode == stand_down) stand_state_check();
    //balance模式状态检查
    if(mode_ctr->mode == balance) balance_state_check();
    //locomotion模式状态检查
    if(mode_ctr->mode == locomotion) locomotion_state_check();
}

/**
 * @description: 机器人指令检查
 * @param {无}
 * @return {无}
 */
void safety_check::run_cmd_check()
{
    //任何上电模式下都进行当前指令的输出力矩检查
    //任何上电模式下都进行当前指令的物理极限关节位置检查
    if(mode_ctr->mode != power_off)
    {
        if(mode_ctr->mode != locomotion) torque_cmd_check();//locomotion模式除外
        jpos_cmd_check();
    }
    //fold模式下指令检查
    if(mode_ctr->mode == fold) fold_cmd_check();
    //stand_up和stand_down模式下指令检查
    if(mode_ctr->mode == stand_up || mode_ctr->mode == stand_down) stand_cmd_check();
    //balance模式指令检查
    if(mode_ctr->mode == balance) balance_cmd_check();
    //locomotion模式指令检查
    if(mode_ctr->mode == locomotion) locomotion_cmd_check();

    if(mode_ctr->mode == estop)//执行estop模式
    {
        //上电无输出
        leg_drv->zero_cmd();
        leg_drv->leg_enable = true;//power on
    }
}

/**
 * @description:  机器人状态安全检查,关节力矩
 * @param {无}
 * @return {无}
 */
void safety_check::torque_state_check()
{
    for(int leg = 0; leg < 4; leg++)
    {
        for(int joint = 0; joint < 3; joint++)
        {
            if(leg_drv->state[leg].tau(joint) > robot::max_tau || leg_drv->state[leg].tau(joint) < -robot::max_tau)
            {
                mode_ctr->mode = estop;
                state_check_code = 1;
                printf("[state check][all mode][tau] leg: %d, joint: %d, tau: %f\n", leg, joint, leg_drv->state[leg].tau(joint));
            }
        }
    }
}

/**
 * @description:  机器人状态安全检查,机身方位
 * @param {无}
 * @return {无}
 */
void safety_check::orientation_state_check()
{
    for(int i = 0; i < 2; i++)
    {
        if(body_data->rpy(i) > 0.5 || body_data->rpy(i) < -0.5)//30度
        {
            mode_ctr->mode = estop;
            state_check_code = 2;
            printf("[state check][all mode][orientation] orientation: %d, angle: %f\n", i, body_data->rpy(i));
        }      
    }
}

/**
 * @description:  机器人状态安全检查,fold模式
 * @param {无}
 * @return {无}
 */
void safety_check::fold_state_check()
{
    const float max_jpos[3] = {1.2,  1.5, 2.8};
    const float min_jpos[3] = {-1.2, -1.6, 0};
    for(int leg = 0; leg < 4; leg++)
    {
        for(int joint = 0; joint < 3; joint++)//关节位置检查
        {
            if(leg_drv->state[leg].q(joint) > max_jpos[joint])
            {
                mode_ctr->mode = estop;
                state_check_code = 3;
                printf("[state check][fold][q][max] leg: %d, joint: %d, q: %f\n", leg, joint, leg_drv->state[leg].q(joint));
            }
            if(leg_drv->state[leg].q(joint) < min_jpos[joint])
            {
                mode_ctr->mode = estop;
                state_check_code = 3;
                printf("[state check][fold][q][min] leg: %d, joint: %d, q: %f\n", leg, joint, leg_drv->state[leg].q(joint));
            }
        }
    }
}

/**
 * @description:  机器人状态安全检查,stand模式
 * @param {无}
 * @return {无}
 */
void safety_check::stand_state_check()
{
    const float max_jpos[3] = {0.5,  -0.3, 2.8};
    const float min_jpos[3] = {-0.5, -1.5, 1.1};
    for(int leg = 0; leg < 4; leg++)
    {
        for(int joint = 0; joint < 3; joint++)//关节位置检查
        {
            if(leg_drv->state[leg].q(joint) > max_jpos[joint])
            {
                mode_ctr->mode = estop;
                state_check_code = 4;
                printf("[state check][stand][q][max] leg: %d, joint: %d, q: %f\n", leg, joint, leg_drv->state[leg].q(joint));
            }
            if(leg_drv->state[leg].q(joint) < min_jpos[joint])
            {
                mode_ctr->mode = estop;
                state_check_code = 4;
                printf("[state check][stand][q][min] leg: %d, joint: %d, q: %f\n", leg, joint, leg_drv->state[leg].q(joint));
            }
        }
    }
}

/**
 * @description:  机器人状态安全检查,balance模式
 * @param {无}
 * @return {无}
 */
void safety_check::balance_state_check()
{
    for(int leg = 0; leg < 4; leg++)
    {
        for(int i = 0; i < 3; i++)
        {
            if(leg_drv->state[leg].p(i) > robot::max_fpos[i])
            {
                mode_ctr->mode = estop;
                state_check_code = 5;
                printf("[state check][balance][p][max] leg: %d, orientation: %d, p: %f\n", leg, i, leg_drv->state[leg].p(i));
            }
            if(leg_drv->state[leg].p(i) < robot::min_fpos[i])
            {
                mode_ctr->mode = estop;
                state_check_code = 5;
                printf("[state check][balance][p][min] leg: %d, orientation: %d, p: %f\n", leg, i, leg_drv->state[leg].p(i));
            }
        }
    }
}

/**
 * @description:  机器人状态安全检查,locomotion模式
 * @param {无}
 * @return {无}
 */
void safety_check::locomotion_state_check()
{
    for(int leg = 0; leg < 4; leg++)
    {
        for(int i = 0; i < 3; i++)
        {
            if(leg_drv->state[leg].p(i) > robot::max_fpos[i])
            {
                mode_ctr->mode = estop;
                state_check_code = 6;
                printf("[state check][locomotion][p][max] leg: %d, orientation: %d, p: %f\n", leg, i, leg_drv->state[leg].p(i));
            }
            if(leg_drv->state[leg].p(i) < robot::min_fpos[i])
            {
                mode_ctr->mode = estop;
                state_check_code = 6;
                printf("[state check][locomotion][p][min] leg: %d, orientation: %d, p: %f\n", leg, i, leg_drv->state[leg].p(i));
            }
        }
    }
}




/**
 * @description:  机器人指令检查,关节力矩
 * @param {无}
 * @return {无}
 */
void safety_check::torque_cmd_check()
{
    for(int leg = 0; leg < 4; leg++)
    {
        Vec3<float> tau = leg_drv->cmd[leg].kp * (leg_drv->cmd[leg].q_des  - leg_drv->state[leg].q)  +
                            leg_drv->cmd[leg].kd * (leg_drv->cmd[leg].qd_des - leg_drv->state[leg].qd);
        Vec3<float> tau_all = tau + leg_drv->cmd[leg].tau_ff;
        for(int joint = 0; joint < 3; joint ++)
        {
            if(tau_all(joint) > robot::limit_tau || tau_all(joint) < -robot::limit_tau)
            {
                mode_ctr->mode = estop;
                cmd_check_code = 1;
                printf("[cmd check][all mode][tau] leg: %d, joint: %d, tau: %f\n", leg, joint, tau_all(joint));
            }
            // if(leg_drv->cmd[leg].tau_ff(joint) > robot::limit_tau || leg_drv->cmd[leg].tau_ff(joint) < -robot::limit_tau)
            // {
            //     mode_ctr->mode = estop;
            //     cmd_check_code = 1;
            //     printf("[cmd check][all mode][tau] leg: %d, joint: %d, tau_ff: %f\n", leg, joint, leg_drv->cmd[leg].tau_ff(joint));
            // }
            // if(tau_all(joint) > robot::limit_tau)
            // {
            //     leg_drv->cmd[leg].tau_ff(joint) = robot::limit_tau - tau(joint);
            // }
            // if(tau_all(joint) < -robot::limit_tau)
            // {
            //     leg_drv->cmd[leg].tau_ff(joint) = -robot::limit_tau - tau(joint);
            // }
        }
    }
}

/**
 * @description:  机器人指令检查,关节物理极限位置
 * @param {无}
 * @return {无}
 */
void safety_check::jpos_cmd_check()
{
    for(int leg = 0; leg < 4; leg++)
    {
        for(int joint = 0; joint < 3; joint ++)
        {
            if(leg_drv->cmd[leg].q_des(joint) > robot::max_jpos[joint])
            {
                mode_ctr->mode = estop;
                cmd_check_code = 2;
                printf("[cmd check][all mode][max jpos] leg: %d, joint: %d, q_des: %f\n", leg, joint, leg_drv->cmd[leg].q_des(joint));
            }
            if(leg_drv->cmd[leg].q_des(joint) < robot::min_jpos[joint])
            {
                mode_ctr->mode = estop;
                cmd_check_code = 2;
                printf("[cmd check][all mode][min jpos] leg: %d, joint: %d, q_des: %f\n", leg, joint, leg_drv->cmd[leg].q_des(joint));
            }
        }
    }
}

/**
 * @description:  机器人指令检查,fold模式
 * @param {无}
 * @return {无}
 */
void safety_check::fold_cmd_check()
{
    const float max_jpos[3] = { 1.2,  1.5, 2.8};
    const float min_jpos[3] = {-1.2, -1.6, 0};
    for(int leg = 0; leg < 4; leg++)
    {
        for(int joint = 0; joint < 3; joint++)
        {
            if(leg_drv->cmd[leg].q_des(joint) > max_jpos[joint])
            {
                mode_ctr->mode = estop;
                cmd_check_code = 3;
                printf("[cmd check][fold][q][max] leg: %d, joint: %d, q_des: %f\n", leg, joint, leg_drv->cmd[leg].q_des(joint));
            }
            if(leg_drv->cmd[leg].q_des(joint) < min_jpos[joint])
            {
                mode_ctr->mode = estop;
                cmd_check_code = 3;
                printf("[cmd check][fold][q][min] leg: %d, joint: %d, q_des: %f\n", leg, joint, leg_drv->cmd[leg].q_des(joint));
            }
        }
    }
}

/**
 * @description:  机器人指令检查,stand模式
 * @param {无}
 * @return {无}
 */
void safety_check::stand_cmd_check()
{
    const float max_jpos[3] = {0.5,  -0.3, 2.8};
    const float min_jpos[3] = {-0.5, -1.5, 1.1};
    for(int leg = 0; leg < 4; leg++)
    {
        for(int joint = 0; joint < 3; joint++)
        {
            if(leg_drv->cmd[leg].q_des(joint) > max_jpos[joint])
            {
                mode_ctr->mode = estop;
                cmd_check_code = 4;
                printf("[cmd check][stand][q][max] leg: %d, joint: %d, q_des: %f\n", leg, joint, leg_drv->cmd[leg].q_des(joint));
            }
            if(leg_drv->cmd[leg].q_des(joint) < min_jpos[joint])
            {
                mode_ctr->mode = estop;
                cmd_check_code = 4;
                printf("[cmd check][stand][q][min] leg: %d, joint: %d, q_des: %f\n", leg, joint, leg_drv->cmd[leg].q_des(joint));
            }
        }
    }
}

/**
 * @description:  机器人指令检查,balance模式
 * @param {无}
 * @return {无}
 */
void safety_check::balance_cmd_check()
{
    for(int leg = 0; leg < 4; leg++)
    {
        Mat3<float>* J = NULL;//不需要计算雅克比，所以赋值为空指针
        Vec3<float> p;
        robot::J_FK(leg_drv->cmd[leg].q_des, J, &p, leg);//计算当前指令的足端位置
        for(int i = 0; i < 3; i++)
        {
            if(p(i) > robot::max_fpos[i])
            {
                mode_ctr->mode = estop;
                cmd_check_code = 5;
                printf("[cmd check][balance][p][max] leg: %d, orientation: %d, p: %f\n", leg, i, p(i));
            }
            if(p(i) < robot::min_fpos[i])
            {
                mode_ctr->mode = estop;
                cmd_check_code = 5;
                printf("[cmd check][balance][p][min] leg: %d, orientation: %d, p: %f\n", leg, i, p(i));
            }
        }
    }
}

/**
 * @description:  机器人指令检查,locomotion模式
 * @param {无}
 * @return {无}
 */
void safety_check::locomotion_cmd_check()
{
    for(int leg = 0; leg < 4; leg++)
    {
        if(leg_drv->cmd[leg].kp(0, 0) > 0)//若开启位置控制
        {
            Mat3<float>* J = NULL;//不需要计算雅克比，所以赋值为空指针
            Vec3<float> p;
            robot::J_FK(leg_drv->cmd[leg].q_des, J, &p, leg);//计算当前指令的足端位置
            for(int i = 0; i < 3; i++)
            {
                if(p(i) > robot::max_fpos[i])
                {
                    mode_ctr->mode = estop;
                    cmd_check_code = 6;
                    printf("[cmd check][locomotion][p][max] leg: %d, orientation: %d, p: %f\n", leg, i, p(i));
                }
                if(p(i) < robot::min_fpos[i])
                {
                    mode_ctr->mode = estop;
                    cmd_check_code = 6;
                    printf("[cmd check][locomotion][p][min] leg: %d, orientation: %d, p: %f\n", leg, i, p(i));
                }
            }
        }
    }
}


