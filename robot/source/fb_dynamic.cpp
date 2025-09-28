/*
 * @Author: sun
 * @Date: 2021-05-18 23:30:23
 * @LastEditTime: 2024-10-08 17:12:52
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /robot_software/robot/source/fb_dynamic.cpp
 */

#include "robot.h"
#include "fb_dynamic.h"
#include "vector_tools.h"
#include "Utilities/pseudoInverse.h"
#include <math.h>
#include <Utilities/Timer.h> //记录程序运行时间

using namespace VECT;//向量代数运算工具

/**
 * @description: 构造函数
 * @param {无}
 */
fb_dynamic::fb_dynamic(robot_model* _model, leg_state_data* _state, body_state_estimator_data* _body_data)
{
  model = _model;
  state = _state;
  body_data = _body_data;
}

/**
 * @description: 析构函数
 * @param {无}
 */
fb_dynamic::~fb_dynamic()
{

}

/**
 * @description: 初始化
 * @param {无}
 */
void fb_dynamic::init()
{
    qdd = Vec18<float>::Zero();

    Cj  = Vec12<float>::Zero();
    pc0 = Vec6<float>::Zero();
    Ic0 = Mat6<float>::Zero();
    Hj  = Mat12<float>::Zero();
    F   = Mat6_12<float>::Zero();

    T = Vec18<float>::Zero();
    C = Vec18<float>::Zero();
    H = Mat18<float>::Zero();
    for(int leg = 0; leg < 4; leg++)
    {
      Fc[leg] = Vec3<float>::Zero();
    }

    for(int i = 0; i < 13; i++)
    {
      q[i] = 0;
      qd[i] = 0;

      X[i]   = Mat6<float>::Zero();
      Xr[i]  = Mat6<float>::Zero();
      Xb[i]  = Mat6<float>::Zero();
      Xrb[i] = Mat6<float>::Zero();
      v[i]   = Vec6<float>::Zero();
      vr[i]  = Vec6<float>::Zero();
      Ic[i]  = Mat6<float>::Zero();
      avp[i] = Vec6<float>::Zero();
      avpr[i]= Vec6<float>::Zero();
      f[i]   = Vec6<float>::Zero();
      fr[i]  = Vec6<float>::Zero();
      ar[i]  = Vec6<float>::Zero();
      arr[i] = Vec6<float>::Zero();
      pc[i]  = Vec6<float>::Zero();
      pr[i] = Vec6<float>::Zero();
    }

    //WBC计算相关参数初始化
    for(int i(0); i < 13; i++){
      WBC_X[i]   = Mat6<float>::Zero();
      WBC_X_b[i] = Mat6<float>::Zero();
      WBC_v[i]   = Vec6<float>::Zero();
      WBC_a[i]   = Vec6<float>::Zero();

      WBC_rou_X[i] = Mat6<float>::Zero();
    }

    for(int i(0); i < 4; i++){
      b_J[i]     = Mat6_18<float>::Zero();
      WBC_X_c[i] = Mat6<float>::Zero();
      WBC_X_q[i] = Mat6<float>::Zero();
      q_b_J[i]   = Mat3_18<float>::Zero();
      WBC_q_a[i] = Vec6<float>::Zero();
      WBC_q_v[i] = Vec6<float>::Zero();
      WBC_o_a[i] = Vec3<float>::Zero();
    }

    


}

/**
 * @description: 更新机器人数据到动力学参数
 * @param {无}
 */
void fb_dynamic::update_data()
{
///////////////////////////关节位置和速度
    for(int leg = 0; leg < 4; leg++)
    {
      for(int j = 0; j < 3; j++)
      {
        q[leg * 3 + j + 1]   = state[leg].q(j);
        qd[leg * 3 + j + 1]  = state[leg].qd(j);
      }
    }    
//////////////////////////基座动力学参数
    //该部分注释掉是因为传感器测的加速度中包含重力加速度，即传感器测的加速度为：a-g
    //
    // Vec6<float> ag;
    // ag << 0, 0, 0, 0, 0, -9.81;
    // X[0] = rot(body_data->r_body) * xlt(body_data->position);
    // avp[0] = - X[0] * ag;
    
    //机身速度
    v[0].block(0,0,3,1) = body_data->omega_body;
    v[0].block(3,0,3,1) = body_data->v_body;
    vr[0] = v[0];
//////////////////////////广义加速度
    qdd.block(0, 0, 3, 1) = body_data->alpha_body;
    qdd.block(3, 0, 3, 1) = body_data->a_body;
    for(int leg = 0; leg < 4; leg++)
    {
      for(int j = 0; j < 3; j++)
      {
        qdd(leg * 3 + j + 6)  = state[leg].qdd(j);
      }
    }
}

/**
 * @description: 计算正动力学
 * @param {无}
 */
void fb_dynamic::run_fd()
{
  update_data();

  Timer dynamic_dt;
////////////////////////科里奥利力+重力///////////////////////////////
    for(int i = 1; i <= 12; i++)
    {
      Vec6<float> vj = model->S * qd[i];//关节速度
      Vec6<float> vrj = model->S * qd[i] * model->k[(i-1)%3];//转子
      
      X[i] = model->Xj(q[i]) * model->Xt[i];//物体i到父物体的变换矩阵
      Xr[i] = model->Xj(q[i] * model->k[(i-1)%3]) * model->Xr[i];//转子
      if(model->p[i] == 0)//父物体为基座
      {
        Xb[i] = X[i];//物体i到基物体的变换矩阵
        Xrb[i] = Xr[i];//转子
      }
      else //父物体非基座
      {
        Xb[i] = X[i] * Xb[model->p[i]];//物体i到基物体的变换矩阵
        Xrb[i] = Xr[i] * Xb[model->p[i]];//转子，转子i的父物体是物体i的父物体，不是上一个转子
      }
      v[i] = X[i] * v[model->p[i]] + vj;//物体i的速度
      vr[i] = Xr[i] * v[model->p[i]] + vrj;//转子，转子i的父物体是物体i的父物体，不是上一个转子
      avp[i] = X[i] * avp[model->p[i]] + cpsm(v[i]) * vj;//物体i的加速度
      avpr[i] = Xr[i] * avp[model->p[i]] + cpsm(vr[i]) * vrj;//转子，转子i的父物体是物体i的父物体，不是上一个转子
      f[i] = model->I[i] * avp[i] + cpsf(v[i]) * model->I[i] * v[i];//物体i的力
      fr[i] = model->Ir * avpr[i] + cpsf(vr[i]) * model->Ir * vr[i];//转子
    }
    f[0] = model->I[0] * avp[0] + cpsf(v[0]) * model->I[0] * v[0];//基座的力

    for(int i = 12; i >=1; i--)//从末端开始，将力向上累积
    {
      Cj[i-1] = model->S.transpose() * f[i]; //关节i力
      Cj[i-1] += model->S.transpose() * fr[i];//转子
      f[model->p[i]] += X[i].transpose() * f[i]; //物体i的父物体力
      f[model->p[i]] += Xr[i].transpose() * fr[i];//转子
    }
    pc0 = f[0];

    time_C = dynamic_dt.getMs(); //记录C项计算时间

///////////////////////////////惯性矩阵///////////////////////////////
    Hj = Mat12<float>::Zero();//驱动自由度质量矩阵清零
    for(int i = 0; i <= 12; i++) Ic[i] = model->I[i];//组合惯性张量矩阵初值
    for(int i = 12; i >= 1; i--)
    {
      Ic[model->p[i]] += X[i].transpose() * Ic[i] * X[i]; //组合惯性张量矩阵
      Ic[model->p[i]] += Xr[i].transpose() * model->Ir * Xr[i];
      F.col(i - 1) = Ic[i] * model->S;//物体i的惯性
      Vec6<float> Fr = model->Ir * model->S;//转子惯性
      Hj(i - 1, i - 1) = model->S.transpose() * F.col(i - 1);//关节i惯性
      Hj(i - 1, i - 1) += model->S.transpose() * Fr;//转子
      int j = i;
      while(model->p[j] > 0)
      {
        if(j == i)//初次坐标变换
        {
          F.col(i - 1) = X[j].transpose() * F.col(i - 1);//物体i惯性和转子惯性向上变换
          F.col(i - 1) += Xr[j].transpose() * Fr;
        }
        else
        {
          F.col(i - 1) = X[j].transpose() * F.col(i - 1);//后续变换已经包含初次的转子
        }
        j = model->p[j];
        Hj(i - 1, j - 1) = F.col(i - 1).transpose() * model->S;
        Hj(j - 1, i - 1) = Hj(i - 1, j - 1);
      }
      F.col(i - 1) = Xb[j].transpose() * F.col(i - 1);
    }
    Ic0 = Ic[0];

    time_H = dynamic_dt.getMs(); //记录H项计算时间

///////////////////////////////整合///////////////////////////////
    C.block(0, 0, 6, 1) = pc0;
    C.block(6, 0, 12, 1) = Cj;
    H.topLeftCorner<6, 6>() = Ic0;
    H.topRightCorner<6, 12>() = F;
    H.bottomLeftCorner<12, 6>() = F.transpose();
    H.bottomRightCorner<12, 12>() = Hj;

    //计算关节的合力
    T = H * qdd + C;

    for(int leg = 0; leg < 4; leg++) //计算的合力赋值给4条腿的Tau(三维向量)
    {
      Tau[leg] = T.block(6 + leg * 3, 0, 3, 1);
    }
}

/**
 * @description: 逆动力学，计算合外力
 * @param {无}
 */
void fb_dynamic::run_id()
{
    update_data();

    ar[0] << 0, 0, 0, 0, 0, -9.81;

    for(int i = 1; i <= 12; i++)
    {
      Vec6<float> vj = model->S * qd[i];//关节速度
      Vec6<float> vrj = model->S * qd[i] * model->k[(i-1)%3];//转子
      X[i] = model->Xj(q[i]) * model->Xt[i];//物体i到父物体的变换矩阵
      Xr[i] = model->Xj(q[i] * model->k[(i-1)%3]) * model->Xr[i];//转子
      if(model->p[i] == 0)//父物体为基座
      {
        Xb[i] = X[i];//物体i到基物体的变换矩阵
        Xrb[i] = Xr[i];//转子
      }
      else //父物体非基座
      {
        Xb[i] = X[i] * Xb[model->p[i]];//物体i到基物体的变换矩阵
        Xrb[i] = Xr[i] * Xb[model->p[i]];//转子，转子i的父物体是物体i的父物体，不是上一个转子
      }
      v[i] = X[i] * v[model->p[i]] + vj;//物体i的速度
      vr[i] = Xr[i] * v[model->p[i]] + vrj;//转子，转子i的父物体是物体i的父物体，不是上一个转子
      ar[i] = X[i] * ar[model->p[i]] + cpsm(v[i]) * vj + model->S * qdd(i+5);//物体i的相对加速度
      arr[i] = Xr[i] * ar[model->p[i]] + cpsm(vr[i]) * vrj + model->S * qdd(i+5) * model->k[(i-1)%3];//转子，转子i的父物体是物体i的父物体，不是上一个转子
      pc[i] = model->I[i] * ar[i] + cpsf(v[i]) * model->I[i] * v[i];
      pr[i] = model->Ir * arr[i] + cpsf(vr[i]) * model->Ir * vr[i];
    }
    for(int i = 0; i <= 12; i++) Ic[i] = model->I[i];//组合惯性张量矩阵初值
    pc[0] = model->I[0] * ar[0] + cpsf(v[0]) * model->I[0] * v[0];//基座的组合力初值

    for(int i = 12; i >=1; i--)//从末端开始，组合力,组合惯性张量矩阵
    {
      Ic[model->p[i]] += X[i].transpose() * Ic[i] * X[i]; //组合惯性张量矩阵
      Ic[model->p[i]] += Xr[i].transpose() * model->Ir * Xr[i];
      pc[model->p[i]] += X[i].transpose() * pc[i]; //物体i的父物体力
      pc[model->p[i]] += Xr[i].transpose() * pr[i];//转子
    }
    Vec6<float> a0 = - Ic[0].inverse() * pc[0];
    for(int i = 1; i <= 12; i++)
    {
      a0 = X[i] * a0;
      T(i+5) = model->S.transpose() * (Ic[i] * a0 + pc[i]);
    }

    for(int leg = 0; leg < 4; leg++)
    {
      Tau[leg] = T.block(6 + leg * 3, 0, 3, 1);
    }
}

/**
 * @description: 计算足端接触力
 * @param {无}
 */
void fb_dynamic::run_cf()
{
  for(int leg = 0; leg < 4; leg++)
  {
    DMat<float> Jti;//雅各比矩阵转置的伪逆
    DMat<float> Jt = state[leg].J.transpose();
    pseudoInverse(Jt, 0.001, Jti);
    Tex[leg] = T.block(6 + leg * 3, 0, 3, 1) - state[leg].tau; //足端接触力 = 合力 - 电机力
    // Fc[leg] = Jti * Tex[leg];
    Fc[leg] = Jti * Vec3<float>(0,0,Tex[leg](2));
    
    //无动力学足端接触力
    Fc0[leg] = Jti * (- state[leg].tau);

    //伪足端接触力
    // Fc_fake[leg] = Jti * (- state[leg].fake_tau);
    // Fc_fake[leg] = Jti * T.block(6 + leg * 3, 0, 3, 1);
    // Fc_fake[leg] = Jti * Vec3<float>(0,0,Tex[leg](2));
  }
}


/********************************WBC********************************/

/**
 * @description: WBC所用正运动学递推参数计算
 *               计算正动力学run_fd()后计算此函数
 * @param {无}
 * @结果-参考于宪元大佬论文P71递推正运动学
 *   Mat6<float> WBC_X[13];//物体i到父物体的变换矩阵-左上标和左下标就差1
 *   Mat6<float> WBC_X_b[13];//物体i到世界系的变换矩阵-左下标一直为世界
 *   Vec6<float> WBC_v[13];//在i坐标系下刚体的刚体空间速度
 *   Vec6<float> WBC_a[13];//在i坐标系下刚体的刚体空间加速度
 */
void fb_dynamic::WBC_kin(){
  //索引代表变换矩阵的左上标，也就是新坐标系
  //世界系到机身（0）的变换矩阵单独处理
  WBC_X_b[0] = rot(body_data->r_body) * xlt(body_data->position);
  WBC_X[0] = WBC_X_b[0];
  
  // WBC_vj[0] = v[0]; //中间量不需要了，结果直接从run_fd()得到了
  WBC_v[0] = v[0]; //来自body_data的角速度和线速度（机身坐标系下）
  
  WBC_a[0] = avp[0]; //0阵-----------------------------------将机身的加速度设置为0
  

  for(int i = 1; i <= 12; i++){
    WBC_X[i] = X[i]; //机身到世界系的变换在循环上边处理（i = 0）
    
    WBC_X_b[i] = WBC_X[i] * WBC_X_b[model->p[i]]; //因为是相对于世界系，与run_fd()中不同
    
    WBC_v[i] = v[i];
    WBC_a[i] = avp[i];//取qddot = 0
  } //递推正运动学参数计算（主要摘抄run_fd()）完毕。
}


/*
  求任务雅１，４:
  先计算草稿纸上写的矩阵
  再建立足底坐标系，跟小腿坐标系同向，也就是说只有平移没旋转，定义WBC_Xc为小腿到足底的坐标变换，是个常值阵
  再构建定向足底系，方向与世界系相同，则与上边相反只有旋转没有平移
  将草稿纸上计算的矩阵转换到定向足底系，取后三行，等待把不同腿的矩阵上下堆叠
*/

void fb_dynamic::WBC_TaskJ(){
  // 0，   1，2，3，    4，5，6，     7，8，9，    10，11，12 

  for(int i(1); i <= WBC_Nc; i++){
    int j = rou[i-1];//迭代的刚体编号 

    WBC_rou_X[j].setIdentity(6,6);
    b_J[i-1].middleCols(j-1+6, 1) = model->S;
    
    while(lamda[j] >= 0){
      WBC_rou_X[lamda[j]] = WBC_rou_X[j] * WBC_X[j];
      j = lamda[j];
      
      if(j == 0){//机身，6列 
        b_J[i-1].middleCols(0, 6) = WBC_rou_X[0] * Eigen::Matrix<float, 6, 6>::Identity(6,6); 
      }
      else{
        b_J[i-1].middleCols(j-1+6, 1) = WBC_rou_X[j] * model->S;
      }
    }
  } //第一部分矩阵b_J计算完成

  for(int i(0); i < WBC_Nc; i++){
    WBC_X_c[i].setIdentity();

    Vec3<float> v_l3;
    v_l3 << 0.20727f, 0, 0;

    WBC_X_c[i].block(3, 0, 3, 3) = -cp3m(v_l3);//各小腿坐标系到足底系的坐标变换阵


    WBC_X_q[i].block(0, 0, 3, 3) = WBC_X_b[rou[i]].block(3,3,3,3).transpose();
    WBC_X_q[i].block(3, 3, 3, 3) = WBC_X_b[rou[i]].block(3,3,3,3).transpose();

    q_b_J[i] = (WBC_X_q[i] * WBC_X_c[i] * b_J[i]).block(3,0,3,18); //待组合的任务矩阵

    /*-----------------JdotQdot项计算--------------------*/

    WBC_q_a[i] = WBC_X_q[i] * WBC_X_c[i] * WBC_a[rou[i]];
    WBC_q_v[i] = WBC_X_q[i] * WBC_X_c[i] * WBC_v[rou[i]];//定向足底坐标系下各小腿的加速度和速度（qddot = 0）
    
    WBC_o_a[i] = WBC_q_a[i].block(3,0,3,1) + cp3m(WBC_q_v[i].block(0,0,3,1)) * WBC_q_v[i].block(3,0,3,1);
  }//初步完成任务矩阵和JdotQdot项计算（在运动控制中随触地脚进行组合）
}
