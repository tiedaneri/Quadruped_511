/*
 * @Author: your name
 * @Date: 2021-03-29 17:49:49
 * @LastEditTime: 2024-07-15 17:06:30
 * @LastEditors: Please set LastEditors
 * @Description: 机身状态估计文件，包括机身姿态估计和位置速度估计
 * @FilePath: /robot_software_1/robot/source/body_state_estimator.cpp
 */

#include "body_state_estimator.h"
#include "robot.h"
#include "Math/orientation_tools.h"

/**
 * @description: 构造函数，初始化body_state_estimator_data（实现为body_data）中的矩阵
 * @param {*}
 * @return {*}
 */
body_state_estimator::body_state_estimator()
{
  body_data.orientation   = Quat<float>::Zero();
  body_data.rpy           = Vec3<float>::Zero();
  body_data.r_body        = RotMat<float>::Zero();
  body_data.omega_body    = Vec3<float>::Zero();
  body_data.omega_world   = Vec3<float>::Zero();
  body_data.alpha_body    = Vec3<float>::Zero();
  body_data.a_body        = Vec3<float>::Zero();
  body_data.a_world       = Vec3<float>::Zero();
  body_data.contact_state = Vec4<float>::Zero();
  body_data.position      = Vec3<float>::Zero();
  body_data.v_world       = Vec3<float>::Zero();
  body_data.v_body        = Vec3<float>::Zero();
}

/**
 * @description: 初始化各参数
 * @param {*}
 * @return {*}
 */
void body_state_estimator::init() 
{
  //默认接触状态
  body_data.contact_state = Vec4<float>(0.5, 0.5, 0.5, 0.5);

  float dt = 0.002;//控制周期
  _xhat.setZero();
  _ps.setZero();
  _vs.setZero();
  _A.setZero();
  _A.block(0, 0, 3, 3)   = Eigen::Matrix<float, 3, 3>::Identity();
  _A.block(0, 3, 3, 3)   = dt * Eigen::Matrix<float, 3, 3>::Identity();
  _A.block(3, 3, 3, 3)   = Eigen::Matrix<float, 3, 3>::Identity();
  _A.block(6, 6, 12, 12) = Eigen::Matrix<float, 12, 12>::Identity();
  _B.setZero();
  _B.block(3, 0, 3, 3) = dt * Eigen::Matrix<float, 3, 3>::Identity();
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> C1(3, 6);
  C1 << Eigen::Matrix<float, 3, 3>::Identity(), Eigen::Matrix<float, 3, 3>::Zero();
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> C2(3, 6);
  C2 << Eigen::Matrix<float, 3, 3>::Zero(), Eigen::Matrix<float, 3, 3>::Identity();
  _C.setZero();
  _C.block(0, 0, 3, 6) = C1;
  _C.block(3, 0, 3, 6) = C1;
  _C.block(6, 0, 3, 6) = C1;
  _C.block(9, 0, 3, 6) = C1;
  _C.block(0, 6, 12, 12) = float(-1) * Eigen::Matrix<float, 12, 12>::Identity();
  _C.block(12, 0, 3, 6) = C2;
  _C.block(15, 0, 3, 6) = C2;
  _C.block(18, 0, 3, 6) = C2;
  _C.block(21, 0, 3, 6) = C2;
  _C(27, 17) = float(1);
  _C(26, 14) = float(1);
  _C(25, 11) = float(1);
  _C(24, 8)  = float(1);
  _P.setIdentity();
  _P = float(100) * _P;
  _Q0.setIdentity();
  _Q0.block(0, 0, 3, 3)   = (dt / 20.f) * Eigen::Matrix<float, 3, 3>::Identity();
  _Q0.block(3, 3, 3, 3)   = (dt * 9.8f / 20.f) * Eigen::Matrix<float, 3, 3>::Identity();
  _Q0.block(6, 6, 12, 12) = dt * Eigen::Matrix<float, 12, 12>::Identity();
  _R0.setIdentity();
}

/**
 * @description: 执行机身状态估计
 * @param {*}
 * @return {*}
 */
void body_state_estimator::run()
{
  run_orientation_eatimator();
  run_pv_eatimator();
}
/**
 * @description: 执行机身姿态状态估计
 * @param {*}
 * @return {*}
 */
void body_state_estimator::run_orientation_eatimator()
{
  Vec3<float> omega_old = body_data.omega_body;//上次角速度数据
  //获取IMU数据
  for(int i = 0; i < 4; i++) body_data.orientation(i) = quat[i];
  for(int i = 0; i < 3; i++)
  {
    body_data.omega_body(i)  = gyro[i];
    body_data.a_body(i)      = acc[i];
  }
  //角速度求导计算角加速度，离散时间有误差，使用低通滤波
  float k = 0.1;//滤波系数
  body_data.alpha_body = (body_data.omega_body - omega_old) * k / robot::dt + body_data.alpha_body * (1 - k);
  
  //第一次运行时获取机身的初始姿态四元数数据
  //后面的四元数为相对初始姿态的变化量
  if(first_run_flag)
  {
    Vec3<float> rpy_ini = ori::quatToRPY(body_data.orientation);
    rpy_ini[0] = 0;
    rpy_ini[1] = 0;
    initial_quat = ori::rpyToQuat(-rpy_ini);
    first_run_flag = false;
  }
  body_data.orientation = ori::quatProduct(initial_quat, body_data.orientation);
  body_data.rpy         = ori::quatToRPY(body_data.orientation);
  body_data.r_body      = ori::quaternionToRotationMatrix(body_data.orientation);
  body_data.omega_world  = body_data.r_body.transpose() * body_data.omega_body;
  body_data.a_world      = body_data.r_body.transpose() * body_data.a_body;
}

/**
 * @description: 执行机身位置状态估计
 * @param {*}
 * @return {*}
 */
void body_state_estimator::run_pv_eatimator()
{
  Eigen::Matrix<float, 18, 18> Q = Eigen::Matrix<float, 18, 18>::Identity();
  Q.block(0, 0, 3, 3)   = _Q0.block(0, 0, 3, 3)   * process_noise_pimu;
  Q.block(3, 3, 3, 3)   = _Q0.block(3, 3, 3, 3)   * process_noise_vimu;
  Q.block(6, 6, 12, 12) = _Q0.block(6, 6, 12, 12) * process_noise_pfoot;

  Eigen::Matrix<float, 28, 28> R = Eigen::Matrix<float, 28, 28>::Identity();
  R.block(0, 0, 12, 12)   = _R0.block(0, 0, 12, 12)   * sensor_noise_pimu_rel_foot;
  R.block(12, 12, 12, 12) = _R0.block(12, 12, 12, 12) * sensor_noise_vimu_rel_foot;
  R.block(24, 24, 4, 4)   = _R0.block(24, 24, 4, 4)   * sensor_noise_zfoot;

  int qindex  = 0;
  int rindex1 = 0;
  int rindex2 = 0;
  int rindex3 = 0;

  Vec3<float> g(0, 0, float(-9.81));
  Mat3<float> Rbod = body_data.r_body.transpose();
  Vec3<float> a = body_data.a_world + g; 
  Vec4<float> pzs = Vec4<float>::Zero();
  Vec4<float> trusts = Vec4<float>::Zero();
  Vec3<float> p0, v0;
  p0 << _xhat[0], _xhat[1], _xhat[2];
  v0 << _xhat[3], _xhat[4], _xhat[5];

  for (int i = 0; i < 4; i++) 
  {
    int i1 = 3 * i;
    Vec3<float> ph = robot::hip_location[i];
    Vec3<float> p_rel = ph + leg_state[i].p;
    Vec3<float> dp_rel = leg_state[i].v;  
    Vec3<float> p_f = Rbod * p_rel;
    Vec3<float> dp_f = Rbod * (body_data.omega_body.cross(p_rel) + dp_rel);

    qindex  = 6 + i1;
    rindex1 = i1;
    rindex2 = 12 + i1;
    rindex3 = 24 + i;

    float trust = float(1);
    float phase = fmin(body_data.contact_state(i), float(1));
    float trust_window = float(0.2);

    if (phase < trust_window) 
    {
      trust = phase / trust_window;
    } 
    else if (phase > (float(1) - trust_window)) 
    {
      trust = (float(1) - phase) / trust_window;
    }
    float high_suspect_number(100);

    Q.block(qindex,  qindex,  3, 3) = (float(1) + (float(1) - trust) * high_suspect_number) * Q.block(qindex, qindex, 3, 3);
    R.block(rindex1, rindex1, 3, 3) = 1 * R.block(rindex1, rindex1, 3, 3);
    R.block(rindex2, rindex2, 3, 3) = (float(1) + (float(1) - trust) * high_suspect_number) * R.block(rindex2, rindex2, 3, 3);
    R(rindex3, rindex3) = (float(1) + (float(1) - trust) * high_suspect_number) * R(rindex3, rindex3);
    trusts(i) = trust;

    _ps.segment(i1, 3) = -p_f;
    _vs.segment(i1, 3) = (1.0f - trust) * v0 + trust * (-dp_f);
    pzs(i) = (1.0f - trust) * (p0(2) + p_f(2));
  }

  Eigen::Matrix<float, 28, 1> y;
  y << _ps, _vs, pzs;
  _xhat = _A * _xhat + _B * a;
  Eigen::Matrix<float, 18, 18> At = _A.transpose();
  Eigen::Matrix<float, 18, 18> Pm = _A * _P * At + Q;
  Eigen::Matrix<float, 18, 28> Ct = _C.transpose();
  Eigen::Matrix<float, 28, 1> yModel = _C * _xhat;
  Eigen::Matrix<float, 28, 1> ey = y - yModel;
  Eigen::Matrix<float, 28, 28> S = _C * Pm * Ct + R;

  Eigen::Matrix<float, 28, 1> S_ey = S.lu().solve(ey);
  _xhat += Pm * Ct * S_ey;

  Eigen::Matrix<float, 28, 18> S_C = S.lu().solve(_C);
  _P = (Eigen::Matrix<float, 18, 18>::Identity() - Pm * Ct * S_C) * Pm;

  Eigen::Matrix<float, 18, 18> Pt = _P.transpose();
  _P = (_P + Pt) / float(2);

  if (_P.block(0, 0, 2, 2).determinant() > float(0.000001))
  {
    _P.block(0, 2, 2, 16).setZero();
    _P.block(2, 0, 16, 2).setZero();
    _P.block(0, 0, 2, 2) /= float(10);
  }

  body_data.position  = _xhat.block(0, 0, 3, 1);
  body_data.v_world   = _xhat.block(3, 0, 3, 1);
  body_data.v_body    = body_data.r_body * body_data.v_world;
}
