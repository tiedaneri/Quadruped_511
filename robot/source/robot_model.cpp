/*
 * @Author: sun
 * @Date: 2021-05-17 18:15:37
 * @LastEditTime: 2024-09-05 16:31:34
 * @LastEditors: Please set LastEditors
 * @Description: 包含机器人的物理参数，创建机器人运动树模型，用于计算动力学
 * @FilePath: /robot_software/robot/source/robot_model.cpp
 */

#include "robot_model.h"
#include "vector_tools.h"
#include <math.h>

using namespace VECT;//向量代数运算工具
/**
 * @description: 构造函数
 * @param {无}
 */
robot_model::robot_model()
{

}

/**
 * @description: 析构函数
 * @param {无}
 */
robot_model::~robot_model()
{

}

/**
 * @description: 创建四足机器人的运动树模型
 * @param {无}
 */
void robot_model::build()
{
  nb = 12;
  k[0] = 6;
  k[1] = 6;
  k[2] = 9.33;
  int parent[13] = {0,0,1,2,0,4,5,0,7,8,0,10,11};
  for(int i = 0; i < 13; i++)
  {
    p[i] = parent[i];
  }
  S << 0, 0, 1, 0, 0, 0; //都是绕z轴旋转

  build_inertia();
  build_tree();

}

/**
 * @description: 计算关节的空间向量变换矩阵
 * @param  {float} q：关节角度
 * @return {Mat6}  X：计算结果
 */
Mat6<float> robot_model::Xj(float q)
{
  Mat3<float> E = rz(-q);
  Mat6<float> X = rot(E);
  return X;
}

/**
 * @description: 创建四足机器人连杆和转子的空间惯量矩阵
 * @param {无}
 */
void robot_model::build_inertia()
{
  double ki = 0.000001;//惯性张量矩阵的系数
  float m;//部件质量
  Vec3<float> c;//质心在部件坐标系下的位置
  Mat3<float> i;//质心处的惯性张量矩阵
  
  m = 3.3;
  c << 0, 0, 0;
  i << 11253, 0, 0, 0, 36203, 0, 0, 0, 42673;
  I[0] = spatial_inertia(m, c, i * ki);//浮动基空间惯量

  m = 0.055;
  c << 0, 0, 0;
  i << 33, 0, 0, 0, 33, 0, 0, 0, 63;
  Ir = spatial_inertia(m, c, i * ki);//转子旋转轴为z轴时的空间惯量

  m = 0.54;
  c << -0.036, 0, 0;
  i << 560, 0.95, -58, 0.95, 444, -0.45, -58, -0.45, 381;
  I[4] = spatial_inertia(m, c, i * ki);//左前abad连杆的空间惯量
  c << -0.036, 0, 0;
  i << 560, -0.95, 58, -0.95, 444, -0.45, 58, -0.45, 381;
  I[10] = spatial_inertia(m, c, i * ki);//左后abad连杆的空间惯量
  c <<  0.036, 0, 0;
  i << 560, 0.95, 58, 0.95, 444, 0.45, 58, 0.45, 381;
  I[1] = spatial_inertia(m, c, i * ki);//右前abad连杆的空间惯量
  c <<  0.036, 0, 0;
  i << 560, -0.95, 58, -0.95, 444, 0.45, -58, 0.45, 381;
  I[7] = spatial_inertia(m, c, i * ki);//右后abad连杆的空间惯量

  m = 0.634;
  c << 0.02, 0, 0.016;
  i << 408, 13, -1.5, 13, 1983, -245, -1.5, -245, 2103;
  I[5] = spatial_inertia(m, c, i * ki);//左前hip连杆的空间惯量
  I[11] = I[5];                         //左后hip连杆的空间惯量
  c << 0.02, 0, -0.016;
  i << 408, -13, 1.5, -13, 1983, -245, 1.5, -245, 2103;
  I[2] = spatial_inertia(m, c, i * ki);//右前hip连杆的空间惯量
  I[8] = I[2];                         //右后hip连杆的空间惯量

  m = 0.064 * 2;
  c << 0.061, 0, 0;
  i << 245, 0, 0, 0, 6, 0, 0, 0, 248;
  i *= 40;
  I[6] = spatial_inertia(m, c, i * ki);//左前knee连杆的空间惯量
  I[12] = I[6];                        //左后knee连杆的空间惯量
  I[3]  = I[6];                        //右前knee连杆的空间惯量
  I[9]  = I[6];                        //右后knee连杆的空间惯量
}

/**
 * @description: 创建四足机器人连杆和转子的运动树空间运动向量变换矩阵
 * @param {无}
 */
void robot_model::build_tree()
{
  Vec3<float> r;//下一坐标系相对上一个坐标系的位置在上一个坐标系下的表示
  Mat3<float> E;//下一坐标系向上一个坐标系进行旋转的旋转矩阵

  Xt[0] = Mat6<float>::Identity();//浮动基

  r << 0.196, -0.05, 0;//abad关节
  E = rx(M_PI_2) * rz(M_PI_2); // --------------- 需要先绕x旋转pi/2，再绕z旋转pi/2
  Xt[1]  = rot(E) * xlt(r);//右前
  r << 0.196, 0.05, 0;
  Xt[4]  = rot(E) * xlt(r);//左前
  r << -0.196, -0.05, 0;
  Xt[7]  = rot(E) * xlt(r);//右后
  r << -0.196, 0.05, 0;
  Xt[10] = rot(E) * xlt(r);//左后

  r << 0.125, -0.05, 0;//abad转子
  Xr[1]  = rot(E) * xlt(r);//右前
  r << 0.125, 0.05, 0;
  Xr[4]  = rot(E) * xlt(r);//左前
  r << -0.125, -0.05, 0;
  Xr[7]  = rot(E) * xlt(r);//右后
  r << -0.125, 0.05, 0;
  Xr[10] = rot(E) * xlt(r);//左后
  
  r << 0.07575, 0, 0;//hip关节
  E = rx(-M_PI_2) * rz(-M_PI_2);// ---------------需要先绕x旋转-pi/2，再绕z旋转-pi/2
  Xt[2] = rot(E) * xlt(r);//右前
  Xt[8] = Xt[2];//右后
  r << -0.07575, 0, 0;
  Xt[5]  = rot(E) * xlt(r);//左前
  Xt[11] = Xt[5];//左后

  r << 0.00225, 0, 0;//hip转子
  Xr[2] = rot(E) * xlt(r);//右前
  Xr[8] = Xr[2];//右后
  r << -0.00225, 0, 0;
  Xr[5]  = rot(E) * xlt(r);//左前
  Xr[11] = Xr[5];//左后

  r << 0.2115, 0, 0;//knee关节
  E = Mat3<float>::Identity();
  Xt[3]  = rot(E) * xlt(r);//右前
  Xt[6]  = Xt[3];//左前
  Xt[9]  = Xt[3];//右后
  Xt[12] = Xt[3];//左后

  r << 0, 0, -0.029;//knee转子
  Xr[3] = rot(E) * xlt(r);//右前
  Xr[9] = Xr[3];//右后
  r << 0, 0, 0.029;
  Xr[6]  = rot(E) * xlt(r);//左前
  Xr[12] = Xr[6];//左后
}




