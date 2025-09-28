/*
 * @Author: your name
 * @Date: 2021-10-11 17:58:42
 * @LastEditTime: 2021-12-01 23:02:22
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /robot_software/robot/source/contact_detection.cpp
 */

#include "contact_detection.h"
#include <math.h>

/**
 * @description: 构造函数
 * @param {无}
 */
contact_detection::contact_detection(bool* _stance_flag, float* _stance_state, float* _swing_state, Vec3<float>* _Fc)
{
  stance_flag  = _stance_flag;
  stance_state = _stance_state;
  swing_state  = _swing_state;

  Fc = _Fc;

  init();
}

/**
 * @description: 析构函数
 * @param {无}
 */
contact_detection::~contact_detection()
{

}

/**
 * @description: 初始化
 * @param {无}
 */
void contact_detection::init()
{
  e_st1 = 0;
  e_st2 = 1;
  e_sw1 = 0;
  e_sw2 = 1;
  sd_st1 = 0.1;
  sd_st2 = 0.1;
  sd_sw1 = 0.1;
  sd_sw2 = 0.1;
  e_f = 15;
  sd_f = 10;
  q = 0.3;
  r = 0.2;
  pt = 0.6;
}

/**
 * @description: 计算步态规划周期预测的触地概率
 * @param {无}
 */
void contact_detection::run()
{
  run_prediction();
  run_measure();
  run_kalman();
}
/**
 * @description: 计算步态规划周期预测的触地概率
 * @param {无}
 */
void contact_detection::run_prediction()
{
  for(int leg = 0; leg < 4; leg++)
  {
    if(stance_flag[leg])//支撑相
    {
      if(stance_state[leg] <= 0.5)//前半段
      {
        pp[leg] = SND((stance_state[leg] - e_st1) / sd_st1);
      }
      else//后半段
      {
        pp[leg] = 1 - SND((stance_state[leg] - e_st2) / sd_st2);
      }
    }
    else//摆动相
    {
      if(swing_state[leg] <= 0.5)//前半段
      {
        pp[leg] = 1 - SND((swing_state[leg] - e_sw1) / sd_sw1);
      }
      else//后半段
      {
        pp[leg] = SND((swing_state[leg] - e_sw2) / sd_sw2);
      }
    }
  }
}

/**
 * @description: 足端力测量的的触地概率
 * @param {无}
 */
void contact_detection::run_measure()
{
  for(int leg = 0; leg < 4; leg++)
  {
    pm[leg] = SND((Fc[leg](2) - e_f) / sd_f);
  }
}

/**
 * @description: 卡尔曼数据融合
 * @param {无}
 */
void contact_detection::run_kalman()
{
  float kk = q / (q + r);
  for(int leg = 0; leg < 4; leg++)
  {
    pc[leg] = pp[leg] + kk * (pm[leg] - pp[leg]);
    if(pc[leg] > pt)
    {
      flag[leg] = true;
    }
    else
    {
      flag[leg] = false;
    }
  }
}


/**
 * @description: 计算标准正态分布
 * @param {无}
 */
float contact_detection::SND(float x)
{
  float y = (float)(0.5 * (1 + erf(x / sqrt(2))));
  return y;
}




