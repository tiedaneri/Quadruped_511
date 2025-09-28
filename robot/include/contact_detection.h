/*
 * @Author: your name
 * @Date: 2021-10-11 17:59:27
 * @LastEditTime: 2021-12-01 23:01:50
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /robot_software/robot/include/contact_detection.h
 */

/*
指南:
      0. 创建对象时自动运行: contact_detection, init;
      1. 运行接触状态检测：run
*/

#ifndef CONTACT_DETECTION_H
#define CONTACT_DETECTION_H

//包含Vec3<float>等与Eigen有关类型的定义
#include "cppTypes.h"

class contact_detection
{
  public:
    //在类中定义Eigen变量时要加入下面这句用于数据对齐
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    contact_detection(bool* _stance_flag, float* _stance_state, float* _swing_state, Vec3<float>* _Fc);
    ~contact_detection();
    
    void init(void);
    void run(void);

    bool flag[4];//足端触地状态
    float pc[4];//卡尔曼数据融合，足端触底概率
    float pp[4];//预测概率
    float pm[4];//测量概率

    //支撑相前半段接触的期望，方差。后半段未接触的期望，标准差
    float e_st1, sd_st1, e_st2, sd_st2;
    //摆动相前半段未接触的期望，方差。后半段接触的期望，标准差
    float e_sw1, sd_sw1, e_sw2, sd_sw2;

    //足端力接触的期望，标准差
    float e_f, sd_f;

    //触地阈值
    float pt;

///////////////////////////////////卡尔曼滤波参数
    float q;//预测噪声协方差矩阵元素
    float r;//测量噪声协方差矩阵元素
    
private:
    void run_prediction(void);
    void run_measure(void);
    void run_kalman(void);

    float SND(float x);
    
    //变量，存储各腿是否处于支撑相
    //true:支撑相，false:摆动相
    bool* stance_flag;
    //变量， 存储支撑状态，0-1
    float* stance_state;
    //变量， 存储摆动状态，0-1
    float* swing_state;

    //变量，足端接触力
    Vec3<float>* Fc;
};

#endif //#ifndef CONTACT_DETECTION_H
