/*
 * @Author: SUN
 * @Date: 2021-03-18 18:07:04
 * @LastEditTime: 2022-03-27 18:59:07
 * @LastEditors: Please set LastEditors
 * @Description: 头文件
 *               保存所有的机器人参数、限位等数据
 *               声明运动学函数
 * @FilePath: /robot_software/robot/include/robot.h
 */

#ifndef ROBOT_H
#define ROBOT_H

//包含变量类型：Vec3等
#include "cppTypes.h"

#ifdef ROBOT1
#define ROBOT_K 1    //机器人1电机控制系数
#endif
#ifdef ROBOT2
#define ROBOT_K  0.75 //机器人2电机控制系数
#endif

namespace robot
{
    //机器人控制周期，0.002s
    const float dt = 0.002f;
    //执行一次MPC的时间开销，0.003s，这个不太准确，估计的
    const float mpc_time = 0.003f;
    
    //abad连杆长度，L1
    const float abad_length = 0.07575f;
    //大腿连杆长度，L2
    const float hip_length  = 0.2115f;
    //小腿连杆长度，L3
    const float knee_length = 0.20727f;

    //各腿坐标变换时，abad关节到hip关节做平移变换时的极性
    const float ah_sign[4] = {1, -1, 1, -1};

    //腿的基坐标系相对于机身坐标系的位置
    const Vec3<float> hip_location[4] = 
    {
        Vec3<float>( 0.196, -0.05, 0),
        Vec3<float>( 0.196,  0.05, 0),
        Vec3<float>(-0.196, -0.05, 0),
        Vec3<float>(-0.196,  0.05, 0)
    };

    //关节物理极限输出扭矩：17 Nm
    const float limit_tau = 17.f;
    //关节控制输出扭矩限制：10 Nm
    const float max_tau = 10.f;
    //足端安全速度：2 m/s
    const float max_fv = 2.f;
    //关节安全速度：6 rad/m
    const float max_jv = 6.f;
    //关节物理极限位置
    const float max_jpos[3] = {1.2,  2.8, 2.8};
    const float min_jpos[3] = {-1.2, -2.8, -2.8};
    //运动模式下，足端位置的极限值
    const float max_fpos[3] = { 0.28,  0.28, -0.1};
    const float min_fpos[3] = {-0.28, -0.28, -0.4};

    //收腿动作目标关节位置，所有腿相同
    const Vec3<float> fold_jpos(0, -1.4, 2.7);
    //站立动作目标关节位置，所有腿相同
    const Vec3<float> stand_jpos(0, -0.8, 1.6);
    //站立动作目标足端位置
    const Vec3<float> stand_fpos[4] = 
    {
        Vec3<float>(-0.00303, -0.07575, -0.29176),
        Vec3<float>(-0.00303,  0.07575, -0.29176),
        Vec3<float>(-0.00303, -0.07575, -0.29176),
        Vec3<float>(-0.00303,  0.07575, -0.29176)
    };

    void J_FK(const Vec3<float>& q, Mat3<float>* J, Vec3<float>* p, int leg);
    void IK(const Vec3<float>& p, Vec3<float>* q, int leg);
    
}

#endif //#ifndef ROBOT_H
