/*
 * @Author: sun
 * @Date: 2021-03-29 17:48:30
 * @LastEditTime: 2024-09-02 09:46:10
 * @LastEditors: Please set LastEditors
 * @Description: 机身状态估计头文件，包括机身姿态估计和位置速度估计
 * @FilePath: /robot_software/robot/include/body_state_estimator.h
 */
/*
指南：
     0. 创建对象时自动运行: body_state_eatimator();
     1. 初始化参数： init();
     2. 连接腿状态数据： leg_state;
     3. 每次执行前更新IMU数据： quat[4];gyro[3];acc[3];
     4. 执行状态估计：run();
*/
#ifndef BODY_STATE_ESTIMATOR_H
#define BODY_STATE_ESTIMATOR_H

#include "leg_driver.h"
//包含Vec3<float>等与Eigen有关类型的定义
#include "cppTypes.h"

typedef struct
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  //姿态四元数，世界坐标系下，机身相对于世界坐标系
  Quat<float> orientation;
  //姿态欧拉角，世界坐标系下，机身相对于世界坐标系
  Vec3<float> rpy;
  //旋转变换矩阵3x3，将世界坐标系转换到机身坐标系，即：body = r_body * world
  RotMat<float> r_body;
  //机身角速度，机身坐标系下，机身相对于世界坐标系
  Vec3<float> omega_body;
  //机身角速度，世界坐标系下，机身相对于世界坐标系
  Vec3<float> omega_world;
  //机身角加速度，机身坐标系下，机身相对于世界坐标系
  Vec3<float> alpha_body;
  //机身加速度，机身坐标系下，机身相对于世界坐标系
  Vec3<float> a_body;
  //机身加速度，世界坐标系下，机身相对于世界坐标系
  Vec3<float> a_world;

  //足端接触状态
  Vec4<float> contact_state;

  //机身相对于世界坐标系的位置，在世界坐标系下的表示
  Vec3<float> position;
  Vec3<float> v_world;
  //机身速度，机身坐标系下
  Vec3<float> v_body;

}body_state_estimator_data;

/*
身体状态估计类
*/
class body_state_estimator
{
  friend class robot_console;
  
  public:
  //在类中定义Eigen变量时要加入下面这句用于数据对齐
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    body_state_estimator();

    //变量，存储估计的身体状态结果
    body_state_estimator_data body_data;

  protected:
    void init();
    void run();
    
    //指针变量，存储腿状态数据，由友元类连接数据
    leg_state_data* leg_state;
    //变量，存储IMU数据，由友元类更新值
    float quat[4];
    float gyro[3];
    float  acc[3];

  private:
    void run_orientation_eatimator();
    void run_pv_eatimator();

  ////////////////////////////////////////机身姿态估计的变量

    //变量，第一次运行姿态状态估计的标志
    bool first_run_flag = true;
    //变量，存储初始的机身四元数数据
    Quat<float> initial_quat;

  ////////////////////////////////////////机身位置估计的变量
    Eigen::Matrix<float, 18, 1>  _xhat;
    Eigen::Matrix<float, 12, 1>  _ps;
    Eigen::Matrix<float, 12, 1>  _vs;
    Eigen::Matrix<float, 18, 18> _A;
    Eigen::Matrix<float, 18, 18> _Q0;
    Eigen::Matrix<float, 18, 18> _P;
    Eigen::Matrix<float, 28, 28> _R0;
    Eigen::Matrix<float, 18, 3>  _B;
    Eigen::Matrix<float, 28, 18> _C;

    const float process_noise_pimu = 0.02;
    const float process_noise_vimu = 0.02;
    const float process_noise_pfoot = 0.002;
    const float sensor_noise_pimu_rel_foot = 0.001;
    const float sensor_noise_vimu_rel_foot = 0.1;
    const float sensor_noise_zfoot = 0.001;

};

#endif  //#ifndef BODY_STATE_ESTIMATOR_H
