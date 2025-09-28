/*
 * @Author: sun
 * @Date: 2021-04-09 21:53:04
 * @LastEditTime: 2021-04-15 05:13:53
 * @LastEditors: Please set LastEditors
 * @Description: 贝塞尔曲线插补法源文件
 * @FilePath: /robot_software/robot/source/bezier_interpolation.cpp
 */

#include "bezier_interpolation.h"

/**
 * @description: 一次贝塞尔曲线位置插补（线性插补）
 * @param {float} p0：起点
 * @param {float} p1：终点
 * @param {float} t ：当前时间占总时间的比例（t<[0,1]）
 * @return {float} p：计算结果
 */
float IBZR::i_b1(float p0, float p1, float t)
{
  float p = p0 + (p1 - p0) * t;
  return p;
}

/**
 * @description: 三次贝塞尔曲线位置插补
 * @param {float} p0：起点
 * @param {float} p1：终点
 * @param {float} t ：当前时间占总时间的比例（t<[0,1]）
 * @return {float} p：计算结果
 */
float IBZR::i_b3(float p0, float p1, float t)
{
  float p = p0 + (p1 - p0) * (3 * t * t - 2 * t * t * t);
  return p;
}

/**
 * @description: 三次贝塞尔曲线速度插补
 * @param {float} p0：起点
 * @param {float} p1：终点
 * @param {float} t ：当前时间占总时间的比例（t<[0,1]）
 * @param {float} rt：真实的总时间
 * @return {float} v：计算结果
 */
float IBZR::i_vb3(float p0, float p1, float t, float rt)
{
  float v = (p1 - p0) * (6 * t - 6 * t * t) / rt;
  return v;
}

/**
 * @description: 三次贝塞尔曲线加速度插补
 * @param {float} p0：起点
 * @param {float} p1：终点
 * @param {float} t ：当前时间占总时间的比例（t<[0,1]）
 * @param {float} rt：真实的总时间
 * @return {float} a：计算结果
 */
float IBZR::i_ab3(float p0, float p1, float t, float rt)
{
  float a = (p1 - p0) * (6 - 12 * t) / (rt * rt);
  return a;
}

