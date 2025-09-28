/*
 * @Author: sun
 * @Date: 2021-01-18 09:04:07
 * @LastEditTime: 2021-10-17 00:11:00
 * @LastEditors: Please set LastEditors
 * @Description: 腿驱动源文件
 *               使用Eigen库，支持矩阵运算，
 *               面向上层控制器，连接下层关节驱动
 * @FilePath: /robot_software/robot/source/leg_driver.cpp
 */

#include "leg_driver.h"

/**
 * @description: 腿指令结构体的所有数据清零
 * @param {无}
 * @return {无}
 */
void leg_command_data::zero()
{
  q_des  = Vec3<float>::Zero();
  qd_des = Vec3<float>::Zero();
  tau_ff = Vec3<float>::Zero();
  kp     = Mat3<float>::Zero();
  kd     = Mat3<float>::Zero();
}

/**
 * @description: 腿状态结构体的所有数据清零
 * @param {无}
 * @return {无}
 */
void leg_state_data::zero()
{
  q   = Vec3<float>::Zero();
  qd  = Vec3<float>::Zero();
  qdd = Vec3<float>::Zero();
  tau = Vec3<float>::Zero();
  J   = Mat3<float>::Zero();
  p   = Vec3<float>::Zero();
  v   = Vec3<float>::Zero();
  // f   = Vec3<float>::Zero();
}

/**
 * @description: 构造函数，初始化参数
 * @param {无}
 * @return {无}
 */
leg_driver::leg_driver() 
{
  leg_enable = false;//所有腿断电

}

/**
 * @description: 腿指令清零
 * @param {无}
 * @return {无}
 */
void leg_driver::zero_cmd() 
{
  //腿指令数据清零
  for (int leg = 0; leg < 4; leg++) cmd[leg].zero();
  leg_enable = false;//腿断电
}

/**
 * @description: 更新腿状态信息
 * @param {无}
 * @return {无}
 */
void leg_driver::update_state() 
{
  for (int leg = 0; leg < 4; leg++) 
  {
    //更新关节位置
    state[leg].q(0) = joint_state.q_abad[leg];
    state[leg].q(1) = joint_state.q_hip[leg];
    state[leg].q(2) = joint_state.q_knee[leg];
    //更新关节速度
    state[leg].qd(0) = joint_state.qd_abad[leg];
    state[leg].qd(1) = joint_state.qd_hip[leg];
    state[leg].qd(2) = joint_state.qd_knee[leg];
    //更新加速度
    state[leg].qdd(0) = joint_state.qdd_abad[leg];
    state[leg].qdd(1) = joint_state.qdd_hip[leg];
    state[leg].qdd(2) = joint_state.qdd_knee[leg];
    //更新关节力矩
    state[leg].tau(0) = joint_torque.tau_abad[leg];
    state[leg].tau(1) = joint_torque.tau_hip[leg];
    state[leg].tau(2) = joint_torque.tau_knee[leg];
    //更新雅克比矩阵和足端位置
    robot::J_FK(state[leg].q, &state[leg].J, &state[leg].p, leg);
    //更新足端速度
    state[leg].v = state[leg].J * state[leg].qd;
    // //更新关节向足端投影的力
    // state[leg].f = state[leg].J * state[leg].tau;

    //伪力矩
    state[leg].fake_tau(0) = joint_torque.fake_tau_abad[leg];
    state[leg].fake_tau(1) = joint_torque.fake_tau_hip[leg];
    state[leg].fake_tau(2) = joint_torque.fake_tau_knee[leg];
  }
}

/**
 * @description: 更新腿指令到关节指令
 * @param {无}
 * @return {无}
 */
void leg_driver::update_cmd()
{
  for (int leg = 0; leg < 4; leg++) 
  {
    //关节位置
    joint_cmd.q_des_abad[leg] = cmd[leg].q_des(0);
    joint_cmd.q_des_hip[leg]  = cmd[leg].q_des(1);
    joint_cmd.q_des_knee[leg] = cmd[leg].q_des(2);
    //关节速度
    joint_cmd.qd_des_abad[leg] = cmd[leg].qd_des(0);
    joint_cmd.qd_des_hip[leg]  = cmd[leg].qd_des(1);
    joint_cmd.qd_des_knee[leg] = cmd[leg].qd_des(2);
    //关节前馈力矩
    joint_cmd.tau_abad_ff[leg] = cmd[leg].tau_ff(0);
    joint_cmd.tau_hip_ff[leg]  = cmd[leg].tau_ff(1);
    joint_cmd.tau_knee_ff[leg] = cmd[leg].tau_ff(2);
    //比例控制系数
    joint_cmd.kp_abad[leg] = cmd[leg].kp(0, 0);
    joint_cmd.kp_hip[leg]  = cmd[leg].kp(1, 1);
    joint_cmd.kp_knee[leg] = cmd[leg].kp(2, 2);
    //微分控制系数
    joint_cmd.kd_abad[leg] = cmd[leg].kd(0, 0);
    joint_cmd.kd_hip[leg]  = cmd[leg].kd(1, 1);
    joint_cmd.kd_knee[leg] = cmd[leg].kd(2, 2);
    //使能（上电）
    joint_cmd.flags[leg] = leg_enable ? 1 : 0;
  }
}

