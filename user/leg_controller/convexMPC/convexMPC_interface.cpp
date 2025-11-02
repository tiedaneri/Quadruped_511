/*
 * @Author: your name
 * @Date: 2021-10-04 04:07:08
 * @LastEditTime: 2021-10-04 06:10:05
 * @LastEditors: your name
 * @Description: In User Settings Edit
 * @FilePath: /robot_software/user/leg_controller/convexMPC/convexMPC_interface.cpp
 */
#include "convexMPC_interface.h"
#include "common_types.h"
#include "SolverMPC.h"
#include <eigen3/Eigen/Dense>
#include <pthread.h>
#include <stdio.h>
#include <string.h>

problem_setup problem_configuration;
update_data_t update;
update_data_Fex_t update_Fex;

void setup_problem(double dt, int horizon, double mu, double f_max)
{
  problem_configuration.horizon = horizon;
  problem_configuration.f_max = f_max;
  problem_configuration.mu = mu;
  problem_configuration.dt = dt;

  resize_qp_mats();
}

//inline to motivate gcc to unroll the loop in here.
//将matlab的float point类型转换为C++ float point类型
inline void mfp_to_flt(flt* dst, mfp* src, s32 n_items)
{
  for(s32 i = 0; i < n_items; i++)
    *dst++ = *src++;
}
//将matlab的int类型转换为C++ u8类型
inline void mint_to_u8(u8* dst, mint* src, s32 n_items)
{
  for(s32 i = 0; i < n_items; i++)
    *dst++ = *src++;
}

int first_solved = 0;  //第一次求解完成

/**
 * @brief  带外力的MPC，更新求解MPC的参数，并求解MPC
 * @param p 世界系下机身位置
 * @param v 世界系下机身速度 
 * @param q 世界系下机身姿态，姿态四元数
 * @param w 世界系下机身角速度
 * @param r 世界系下足端位置，注意布局是[x,x,x,x,y,y,y,y,z,z,z,z]
 * @param Fex 世界系下外力的三维向量
 * @param yaw 偏航角，从body_data的rpy中获取
 * @param weights 二次优化标准表达式中Q矩阵的左上角12x12子阵主对角线元素的值
 * @param state_trajectory 参考轨迹，大小取决于预测步长
 * @param alpha 二次优化标准表达式中R矩阵的系数值
 * @param gait mpc一次预测的所有腿支撑状态，1：支撑，0：摆动
 */
void update_Fex_problem_data_floats(float* p, float* v, float* q, float* w, float* Fex,
                                float* r, float yaw, float* weights,
                                float* state_trajectory, float alpha, int* gait){
  update_Fex.alpha = alpha;
  update_Fex.yaw = yaw;
  mint_to_u8(update_Fex.gait,gait,4*problem_configuration.horizon);
  memcpy((void*)update_Fex.p,(void*)p,sizeof(float)*3);
  memcpy((void*)update_Fex.v,(void*)v,sizeof(float)*3);
  memcpy((void*)update_Fex.q,(void*)q,sizeof(float)*4);
  memcpy((void*)update_Fex.w,(void*)w,sizeof(float)*3);
  memcpy((void*)update_Fex.Fex,(void*)Fex,sizeof(float)*3);
  memcpy((void*)update_Fex.r,(void*)r,sizeof(float)*12);
  memcpy((void*)update_Fex.weights,(void*)weights,sizeof(float)*18);
  memcpy((void*)update_Fex.traj,(void*)state_trajectory, sizeof(float) * 12 * problem_configuration.horizon);

  solve_mpc_2(&update_Fex, &problem_configuration);
  first_solved = 1;
}

/**
 * @brief  无外力MPC，更新求解MPC的参数，并求解MPC
 * @param p 世界系下机身位置
 * @param v 世界系下机身速度 
 * @param q 世界系下机身姿态，姿态四元数
 * @param w 世界系下机身角速度
 * @param r 世界系下足端位置，注意布局是[x,x,x,x,y,y,y,y,z,z,z,z]
 * @param yaw 偏航角，从body_data的rpy中获取
 * @param weights 二次优化标准表达式中Q矩阵的左上角12x12子阵主对角线元素的值
 * @param state_trajectory 参考轨迹，大小取决于预测步长
 * @param alpha 二次优化标准表达式中R矩阵的系数值
 * @param gait mpc一次预测的所有腿支撑状态，1：支撑，0：摆动
 */
void update_problem_data_floats(float* p, float* v, float* q, float* w, 
                                float* r, float yaw, float* weights,
                                float* state_trajectory, float alpha, int* gait){
  update.alpha = alpha;
  update.yaw = yaw;
  mint_to_u8(update.gait,gait,4*problem_configuration.horizon);
  memcpy((void*)update.p,(void*)p,sizeof(float)*3);
  memcpy((void*)update.v,(void*)v,sizeof(float)*3);
  memcpy((void*)update.q,(void*)q,sizeof(float)*4);
  memcpy((void*)update.w,(void*)w,sizeof(float)*3);
  memcpy((void*)update.r,(void*)r,sizeof(float)*12);
  memcpy((void*)update.weights,(void*)weights,sizeof(float)*15);
  memcpy((void*)update.traj,(void*)state_trajectory, sizeof(float) * 12 * problem_configuration.horizon);

  solve_mpc_1(&update, &problem_configuration);
  first_solved = 1;
}

double get_solution(int index)
{
  if(!first_solved) return 0.f;
  
  mfp* qs = get_q_soln();
  return qs[index];
}
