/*
 * @Author: sun
 * @Date: 2021-05-18 23:28:55
 * @LastEditTime: 2024-10-08 16:50:43
 * @LastEditors: Please set LastEditors
 * @Description: 计算浮动基系统动力学表达式
 * @FilePath: /robot_software/robot/include/fb_dynamic.h
 */

/*
指南:
      0. 创建对象时自动运行: fb_dynamic;
      1. 每个控制周期更新数据: build();
      2. 运行动力学功能函数: run_fd()/run_id()/run_cf();
*/

#ifndef FB_DYNAMIC_H
#define FB_DYNAMIC_H

//包含Vec3<float>等与Eigen有关类型的定义
#include "cppTypes.h"
#include "robot_model.h"
#include "leg_driver.h"     //腿驱动
#include "body_state_estimator.h"  //机身状态估计

class fb_dynamic
{
  public:
    //在类中定义Eigen变量时要加入下面这句用于数据对齐
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    fb_dynamic(robot_model* _model, leg_state_data* _state, body_state_estimator_data* _body_data);
    ~fb_dynamic();
    
    void init(void);
    void update_data(void);
    void run_fd(void);
    void run_id(void);
    void run_cf(void);

    double time_C;
    double time_H;
    
    //变量，运动树模型对象
    robot_model* model;
    //变量，存储腿数据
    leg_state_data* state;
    //变量，存储估计的身体状态结果
    body_state_estimator_data* body_data;

///////////////////////////////////////动力学参数
    float q[13];//关节位置
    float qd[13];//关节速度
    Vec18<float> qdd;//广义加速度

///////////////////////////////////////计算结果
    //变量，广义合外力
    Vec18<float> T;
    //变量，科里奥利力+重力
    Vec18<float> C;//正动力学
    //变量，惯性矩阵
    Mat18<float> H;//正动力学

    //变量，合外力矩
    Vec3<float> Tau[4];
    //变量，足端接触外力矩
    Vec3<float> Tex[4];
    //变量，足端接触力
    Vec3<float> Fc[4];
    //变量，无动力学足端接触力
    Vec3<float> Fc0[4];
    //变量，伪足端接触力
    Vec3<float> Fc_fake[4];

  // private:
////////////////////////////正动力学计算结果变量
    Vec12<float> Cj;//关节科氏力
    Vec6<float> pc0;//机身科氏力
    Mat6<float> Ic0;//机身自由度质量矩阵
    Mat12<float> Hj;//驱动自由度质量矩阵
    Mat6_12<float> F;//驱动自由度与机身自由度交叉质量矩阵
    
////////////////////////////计算过程中间变量
    Mat6<float> X[13];//物体i到父物体的变换矩阵
    Mat6<float> Xr[13];//转子
    Mat6<float> Xb[13];//物体i到基物体的变换矩阵
    Mat6<float> Xrb[13];//转子
    Vec6<float> v[13];//物体i的速度
    Vec6<float> vr[13];//转子
    Mat6<float> Ic[13];//组合惯性张量矩阵
    ////正动力学
    Vec6<float> avp[13];//物体i的加速度
    Vec6<float> avpr[13];//转子
    Vec6<float> f[13];//物体i的力
    Vec6<float> fr[13];//转子
    ////逆动力学
    Vec6<float> ar[13];//物体i的相对加速度
    Vec6<float> arr[13];//转子
    Vec6<float> pc[13];//物体i的组合力
    Vec6<float> pr[13];//转子i的力

////////////////////////////WBC正运动学递推////////////////////////////
    void WBC_kin(void);//WBC所用正运动学递推参数计算   

    //结果
    Mat6<float> WBC_X[13];//物体i到父物体的变换矩阵-左上标和左下标就差1
    Mat6<float> WBC_X_b[13];//物体i到世界系的变换矩阵-左下标一直为世界
    // Vec6<float> WBC_vj[13]; //在i坐标系下的关节速度
    Vec6<float> WBC_v[13];//在i坐标系下刚体空间速度
    Vec6<float> WBC_a[13];//在i坐标系下刚体空间加速度

    void WBC_TaskJ(void);//WBC计算任务雅各比和JdotQdot

    int WBC_Nc = 4;//４个接触足
    int rou[4] = {3, 6, 9, 12}; //第i个足所刚接的刚体编号，即小腿编号
    int lamda[13] = {-1,0,1,2,0,4,5,0,7,8,0,10,11};//存储在ＷＢＣ中使用的父连杆编号

    Mat6_18<float> b_J[4];//各小腿的b_J
    Mat6<float> WBC_rou_X[13]; //中间量,在小腿坐标系下到其他刚体的变换矩阵

    Mat6<float> WBC_X_c[4];//各小腿坐标系到足底系的坐标变换阵，四个都相同且只有平移
    Mat6<float> WBC_X_q[4];//各足底系到定向足底系的坐标变换阵，只有旋转

    Mat3_18<float> q_b_J[4]; //待组合的任务矩阵！！！！！！！！！

    Vec6<float> WBC_q_a[4];
    Vec6<float> WBC_q_v[4];//定向足底坐标系下各小腿的加速度和速度（qddot = 0）

    Vec3<float> WBC_o_a[4];//世界系下的足底点的加速度（qddot = 0）－－即JdotQdot！！！！！！！！！
  
};

#endif //#ifndef ROBOT_MODEL_H
