/*
 * @Author: sun
 * @Date: 2021-05-17 23:17:26
 * @LastEditTime: 2021-05-24 00:01:31
 * @LastEditors: Please set LastEditors
 * @Description: 包括三维向量和空间向量的运算工具
 * @FilePath: /robot_software/robot/include/vector_tools.h
 */

#ifndef VECTOR_TOOLS_H
#define VECTOR_TOOLS_H

//包含Vec3<float>等与Eigen有关类型的定义
#include "cppTypes.h"

namespace VECT
{
    Mat6<float> spatial_inertia(float m, Vec3<float> c, Mat3<float> i);
    Mat6<float> cpsm(Vec6<float> v);
    Mat6<float> cpsf(Vec6<float> v);
    Mat6<float> rot(Mat3<float> E);
    Mat6<float> xlt(Vec3<float> r);

    Mat3<float> cp3m(Vec3<float> v);
    Mat3<float> rx(float a);
    Mat3<float> ry(float a);
    Mat3<float> rz(float a);

}

#endif //#ifndef SPATIAL_VECTOR_TOOLS_H


