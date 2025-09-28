/*
 * @Author: sun
 * @Date: 2021-04-04 19:56:24
 * @LastEditTime: 2024-07-11 09:29:21
 * @LastEditors: Please set LastEditors
 * @Description: 轨迹规划源文件
 * @FilePath: /robot_software/user/leg_controller/source/trajectory_plan.cpp
 */


#include "trajectory_plan.h"
#include "robot.h"
#include "bezier_interpolation.h"

#include <stdio.h>//包含"std::"和"printf"定义
#include <math.h>

/**
 * @description: 构造函数，初始化参数
 * @param {无}
 * @return {无}
 */
trajectory_plan::trajectory_plan()
{
    pos_initial = Vec3<float>::Zero();
    pos_final   = Vec3<float>::Zero();
    p           = Vec3<float>::Zero();
    v           = Vec3<float>::Zero();
    a           = Vec3<float>::Zero();
    height      = 0;
}

/**
 * @description: 运行支撑相轨迹规划
 * @param {float} phase：当前相位
 * @param {float} time ：支撑相的总时间
 * @return {*}
 */
void trajectory_plan::run_st(float phase, float time)
{
    for(int i = 0; i < 3; i++)
    {
        p(i) = IBZR::i_b3(pos_initial(i), pos_final(i), phase);
        v(i) = IBZR::i_vb3(pos_initial(i), pos_final(i), phase, time);
        a(i) = IBZR::i_ab3(pos_initial(i), pos_final(i), phase, time);
    }
}
// void trajectory_plan::run_st(float phase, float time)
// {
//     for(int i = 0; i < 3; i++)
//     {
//         p(i) = IBZR::i_b1(pos_initial(i), pos_final(i), phase);
//     }
// }
/**
 * @description: 运行摆动相轨迹规划
 * @param {float} phase：当前相位
 * @param {float} time ：摆动相的总时间
 * @return {*}
 */
void trajectory_plan::run_sw(float phase, float time)
{
    for(int i = 0; i < 2; i++) //x,y 方向
    {
        p(i) = IBZR::i_b3(pos_initial(i), pos_final(i), phase);
        v(i) = IBZR::i_vb3(pos_initial(i), pos_final(i), phase, time);
        a(i) = IBZR::i_ab3(pos_initial(i), pos_final(i), phase, time);
    }

    //z 方向
    if(phase < 0.5)//上升阶段
    {
        p(2) = IBZR::i_b3(pos_initial(2), pos_initial(2) + height, phase * 2);
        v(2) = IBZR::i_vb3(pos_initial(2), pos_initial(2) + height, phase * 2, time / 2);
        a(2) = IBZR::i_ab3(pos_initial(2), pos_initial(2) + height, phase * 2, time / 2);     
    }
    else//phase > 0.5,下降阶段
    {
        p(2) = IBZR::i_b3(pos_initial(2) + height, pos_final(2), phase * 2 - 1);
        v(2) = IBZR::i_vb3(pos_initial(2) + height, pos_final(2), phase * 2 - 1, time / 2);
        a(2) = IBZR::i_ab3(pos_initial(2) + height, pos_final(2), phase * 2 - 1, time / 2);     
    }
}
// void trajectory_plan::run_sw(float phase, float time)
// {
//     for(int i = 0; i < 2; i++)
//     {
//         p(i) = IBZR::i_b1(pos_initial(i), pos_final(i), phase);
//     }
//     if(phase < 0.5)//上升阶段
//     {
//         p(2) = IBZR::i_b1(pos_initial(2), pos_initial(2) + height, phase * 2);     
//     }
//     else//phase > 0.5,下降阶段
//     {
//         p(2) = IBZR::i_b1(pos_initial(2) + height, pos_final(2), phase * 2 - 1);   
//     }
// }