/*
 * @Author: sun
 * @Date: 2021-04-09 21:49:42
 * @LastEditTime: 2021-04-15 05:07:31
 * @LastEditors: Please set LastEditors
 * @Description: 贝塞尔曲线插补法头文件
 * @FilePath: /robot_software/robot/include/bezier_interpolation.h
 */

#ifndef BEZIER_INTERPOLATION_H
#define BEZIER_INTERPOLATION_H

namespace IBZR
{
    float i_b1(float  p0, float p1, float t);
    float i_b3(float  p0, float p1, float t);
    float i_vb3(float p0, float p1, float t, float rt);
    float i_ab3(float p0, float p1, float t, float rt);
}

#endif //#ifndef BEZIER_INTERPOLATION_H
