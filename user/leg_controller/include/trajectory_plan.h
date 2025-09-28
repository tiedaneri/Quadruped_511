/*
 * @Author: sun
 * @Date: 2021-04-04 19:54:51
 * @LastEditTime: 2021-04-15 05:25:50
 * @LastEditors: Please set LastEditors
 * @Description: 轨迹规划头文件
 * @FilePath: /robot_software/user/leg_controller/include/trajectory_plan.h
 */

/*
指南:
      0. 创建对象时自动运行: trajectory_plan;
      1. 设置起点、终点、抬腿高度（仅摆动相）
      2. 计算轨迹：run_st(),run_sw()
      3. 获取轨迹结果：p,v,a
*/

#ifndef TRAJECTORY_PLAN_H
#define TRAJECTORY_PLAN_H

#include <stdint.h>//包含"uint8_t"定义

#include "cppTypes.h"//包含变量类型：Vec3等

class trajectory_plan
{
  public:
    //在类中定义Eigen变量时要加入下面这句用于数据对齐
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    trajectory_plan(void);

    void run_st(float phase, float time);
    void run_sw(float phase, float time);
    
    //变量，存储轨迹起点
    Vec3<float> pos_initial;
    //变量，存储轨迹终点
    Vec3<float> pos_final;
    //变量，存储抬腿高度，仅用于摆动轨迹
    float height;

    //变量，存储当前计算的位置
    Vec3<float> p;
    //变量，存储当前计算的速度
    Vec3<float> v;
    //变量，存储当前计算的加速度
    Vec3<float> a;

};

#endif //TRAJECTORY_PLAN_H
