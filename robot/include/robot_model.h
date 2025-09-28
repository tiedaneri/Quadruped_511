/*
 * @Author: sun
 * @Date: 2021-05-17 18:16:11
 * @LastEditTime: 2021-10-13 19:28:16
 * @LastEditors: Please set LastEditors
 * @Description: 包含机器人的物理参数，创建机器人运动树模型，用于计算动力学
 * @FilePath: /robot_software/robot/include/robot_model.h
 */

/*
指南:
      0. 创建对象时自动运行: robot_model;
      1. 运行建模函数: build();
*/

#ifndef ROBOT_MODEL_H
#define ROBOT_MODEL_H

//包含Vec3<float>等与Eigen有关类型的定义
#include "cppTypes.h"

class robot_model
{
  public:
    //在类中定义Eigen变量时要加入下面这句用于数据对齐
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    robot_model(void);
    ~robot_model();
    
    void build(void);
    Mat6<float> Xj(float q);

    //变量，存储连杆最大编号
    float nb;
    //变量，存储转子传动比：abad, hip, knee
    float k[3];
    //变量，存储父连杆编号,下标从1开始，0无意义
    int p[13];
    //变量，存储各关节运动向量子空间，关节1-12，各关节相同
    Vec6<float> S;
    //变量，存储各连杆空间惯量,下标从0开始,0为浮动基
    Mat6<float> I[13];
    //变量，存储各转子空间惯量,各转子相同
    Mat6<float> Ir;
    //变量，存储运动树各空间变换矩阵,下标从0开始,0为地面到浮动基关节，为单位阵
    Mat6<float> Xt[13];
    //变量，存储转子运动树各空间变换矩阵,下标从1开始，0无意义
    Mat6<float> Xr[13];

  private:
    void build_inertia(void);
    void build_tree(void);
};

#endif //#ifndef ROBOT_MODEL_H
