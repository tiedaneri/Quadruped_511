/*
 * @Author: sun
 * @Date: 2021-03-21 06:46:53
 * @LastEditTime: 2024-09-19 15:41:50
 * @LastEditors: Please set LastEditors
 * @Description: 源文件
 *               保存所有的机器人参数、限位等数据
 *               声明运动学函数
 * @FilePath: /robot_software/robot/source/robot.cpp
 */

/*
注，坐标系建立方式：
  机器人身体坐标系：
      与身体固结，原点与机身质心重合，
      机器人前进方向为x轴正方向，
      机器人前进方向左侧为y轴正方向，
      机器人上方为z轴正方向。
  所有腿的基坐标系：
      与身体固结，原点与该腿的abad关节旋转轴和髋关节旋转轴的交点重合
      方向与机身坐标系相同
  [0]腿关节坐标系：
    0系：与机身固结，原点与腿基坐标系重合，z轴与旋转轴重合，指向正前方，x轴与髋关节旋转轴重合，指向髋关节
    1系：与连杆1（abad-hip连杆）固结，原点与足端在髋关节旋转轴上的投影重合，z轴与髋关节旋转轴重合，远离abad关节方向，x轴指向膝关节
    2系：与连杆2（hip-knee连杆）固结，原点与足端在膝关节旋转轴上的投影重合，z轴与膝关节旋转轴重合，远离abad关节方向，x轴指向足端
    3系：与连杆3（knee-foot连杆）固结，原点与足端重合，z轴与2系平行，方向相同，x轴由膝关节指向足端
  [1-3]腿各关节坐标系建立原点的规则与[0]腿相同，坐标系方向直接与[0]腿相同
    因此，各腿的坐标变换只有0系到1系平移变换时的方向不同，其他变换完全相同
    用s={1，-1, 1,-1}表示该平移的方向，各腿的坐标系变换为：
      Tb0 = trotz(-pi/2)*trotx(-pi/2);
      T01 = trotz(a1)*transx(s*l1)*trotz(pi/2)*trotx(pi/2);
      T12 = trotz(a2)*transx(l2);
      T23 = trotz(a3)*transx(l3);
*/
#include "robot.h"

#include <stdio.h>
#include <math.h>

/**
 * @description: 根据关节位置计算单腿的雅克比矩阵和足端位置（正运动学）
 * @param {
 *         q：  输入的关节位置
 *         J：  输出的雅克比矩阵
 *         p：  输出的足端位置
 *         leg：输入的腿序号
 *        }
 * @return {无}
 */

void robot::J_FK(const Vec3<float>& q, Mat3<float>* J, Vec3<float>* p, int leg)
{
  //各关节连杆长度
  float l1 = abad_length;
  float l2 = hip_length;
  float l3 = knee_length;

  //当前腿坐标变换时，abad关节到hip关节做平移变换时的极性
  float s = ah_sign[leg];

  //各关节角度的sin值
  float s1 = std::sin(q(0));
  float s2 = std::sin(q(1));
  float s3 = std::sin(q(2));

  //各关节角度的cos值
  float c1 = std::cos(q(0));
  float c2 = std::cos(q(1));
  float c3 = std::cos(q(2));

  //关节2与关节3之和的cos值
  float c23 = c2 * c3 - s2 * s3;
  //关节2与关节3之和的sin值
  float s23 = s2 * c3 + c2 * s3;

  //计算雅克比矩阵，只取平动雅
  if(J != NULL)
  {
    J->operator()(0, 0) =  0;
    J->operator()(0, 1) =  l3 * c23 + l2 * c2;
    J->operator()(0, 2) =  l3 * c23;
    J->operator()(1, 0) =  l3 * c1 * c23 + l2 * c1 * c2 + l1 * s * s1;
    J->operator()(1, 1) = -l3 * s1 * s23 - l2 * s1 * s2;
    J->operator()(1, 2) = -l3 * s1 * s23;
    J->operator()(2, 0) =  l3 * s1 * c23 + l2 * c2 * s1 - l1 * s * c1;
    J->operator()(2, 1) =  l3 * c1 * s23 + l2 * c1 * s2;
    J->operator()(2, 2) =  l3 * c1 * s23;
  }

  //正运动学计算足端位置
  if(p != NULL)
  {
    p->operator()(0) =  l3 * s23 + l2 * s2;
    p->operator()(1) = -l1 * s * c1 + l3 * (s1 * c23) + l2 * c2 * s1;
    p->operator()(2) = -l1 * s * s1 - l3 * (c1 * c23) - l2 * c1 * c2;
  }
}

/**
 * @description: 计算机器人的运动学逆解 
 * @param p  ：输入，足端位置
 * @param q  ：输出，关节角度
 * @param leg：输入，腿编号
 * @return {*}
 */
void robot::IK(const Vec3<float>& p, Vec3<float>* q, int leg)
{
  //各关节连杆长度
  float l1 = abad_length;
  float l2 = hip_length;
  float l3 = knee_length;
  //足端位置
  float x = p(0);
  float y = p(1);
  float z = p(2);
  //当前腿坐标变换时，abad关节到hip关节做平移变换时的极性
  float s = ah_sign[leg];

  //计算逆解
  float c1  = -l1*s/sqrt(y*y+z*z);
  float s1  = -sqrt(1-c1*c1);
  float aa1 = atan2(z,y)-atan2(s1,c1);

  float c3  = (x*x+y*y+z*z-l1*l1-l2*l2-l3*l3)/(2*l2*l3);
  float s3  = sqrt(1-c3*c3);
  float aa3 = atan2(s3,c3);

  float a   = l3*c3+l2;
  float b   = l3*s3;
  float c2  = (y*sin(aa1)-z*cos(aa1)+x*b/a)/(a+b*b/a);
  float s2  = (x-b*c2)/a;
  float aa2 = atan2(s2,c2);

  //返回角度值
  q->operator()(0) = aa1;
  q->operator()(1) = aa2;
  q->operator()(2) = aa3;
}

