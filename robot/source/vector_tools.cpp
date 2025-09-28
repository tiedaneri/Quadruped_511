/*
 * @Author: sun
 * @Date: 2021-05-17 23:19:18
 * @LastEditTime: 2021-05-24 00:00:46
 * @LastEditors: Please set LastEditors
 * @Description: 包括三维向量和空间向量的运算工具
 * @FilePath: /robot_software/robot/source/vector_tools.cpp
 */

#include "vector_tools.h"
#include <math.h>

/**
 * @description: 计算空间惯量矩阵
 * @param  {float} m：质量
 * @param  {Vec3}  c：质心位置在物体坐标系下的表示
 * @param  {Mat3}  i：质心处惯性张量矩阵
 * @return {Mat6}  I：计算结果
 */
Mat6<float> VECT::spatial_inertia(float m, Vec3<float> c, Mat3<float> i)
{
    Mat6<float> I;
    Mat3<float> a = cp3m(c);
    I.topLeftCorner<3, 3>() = i + m * a * a.transpose();
    I.topRightCorner<3, 3>() = m * a;
    I.bottomLeftCorner<3, 3>() = m * a.transpose();
    I.bottomRightCorner<3, 3>() = m * Mat3<float>::Identity();
    return I;
}

/**
 * @description: 计算空间运动向量的叉乘矩阵
 * @param  {Vec6}  v：要转换的空间向量
 * @return {Mat6}  m：计算结果
 */
Mat6<float> VECT::cpsm(Vec6<float> v)
{
    Mat6<float> m;
    m << 0,   -v(2), v(1),  0,    0,    0, 
         v(2), 0,   -v(0),  0,    0,    0, 
        -v(1), v(0), 0,     0,    0,    0,
         0,   -v(5), v(4),  0,   -v(2), v(1), 
         v(5), 0,   -v(3),  v(2), 0,   -v(0), 
        -v(4), v(3), 0,    -v(1), v(0), 0;
    return m;
}

/**
 * @description: 计算空间力向量的叉乘矩阵
 * @param  {Vec6}  v：要转换的空间向量
 * @return {Mat6}  m：计算结果
 */
Mat6<float> VECT::cpsf(Vec6<float> v)
{
    Mat6<float> m;
    m << 0,   -v(2), v(1), 0,   -v(5), v(4), 
         v(2), 0,   -v(0), v(5), 0,   -v(3), 
        -v(1), v(0), 0,   -v(4), v(3), 0, 
         0,    0,    0,    0,   -v(2), v(1), 
         0,    0,    0,    v(2), 0,   -v(0),
         0,    0,    0,   -v(1), v(0), 0;
    return m;
}

/**
 * @description: 计算三维向量的叉乘矩阵
 * @param  {Vec3}  v：要转换的三维向量
 * @return {Mat3}  m：计算结果
 */
Mat3<float> VECT::cp3m(Vec3<float> v)
{
    Mat3<float> m;
    m << 0,   -v(2), v(1),
         v(2), 0,   -v(0),
        -v(1), v(0), 0;
    return m;
}

/**
 * @description: 计算空间向量的旋转变换矩阵
 * @param  {Mat3}  E：三维坐标旋转矩阵
 * @return {Mat6}  X：计算结果
 */
Mat6<float> VECT::rot(Mat3<float> E)
{
    Mat6<float> X = Mat6<float>::Zero();
    X.topLeftCorner<3, 3>() = E;
    X.bottomRightCorner<3, 3>() = E;
    return X;
}

/**
 * @description: 计算空间向量的平移变换矩阵
 * @param  {Vec3}  r：三维坐标平移向量
 * @return {Mat6}  X：计算结果
 */
Mat6<float> VECT::xlt(Vec3<float> r)
{
    Mat6<float> X = Mat6<float>::Identity();
    Mat3<float> a = -cp3m(r);
    X.bottomLeftCorner<3, 3>() = a;
    return X;
}

/**
 * @description: 计算三维向量绕x轴的旋转矩阵
 * @param  {float} a：旋转角度
 * @return {Mat3}  E：计算结果
 */
Mat3<float> VECT::rx(float a)
{
    float s = sin(a);
    float c = cos(a);
    Mat3<float> E;
    E << 1, 0, 0, 0, c, -s, 0, s, c;
    return E;
}

/**
 * @description: 计算三维向量绕y轴的旋转矩阵
 * @param  {float} a：旋转角度
 * @return {Mat3}  E：计算结果
 */
Mat3<float> VECT::ry(float a)
{
    float s = sin(a);
    float c = cos(a);
    Mat3<float> E;
    E << c, 0, s, 0, 1, 0, -s, 0, c;
    return E;
}

/**
 * @description: 计算三维向量绕z轴的旋转矩阵
 * @param  {float} a：旋转角度
 * @return {Mat3}  E：计算结果
 */
Mat3<float> VECT::rz(float a)
{
    float s = sin(a);
    float c = cos(a);
    Mat3<float> E;
    E << c, -s, 0, s, c, 0, 0, 0, 1;
    return E;
}












