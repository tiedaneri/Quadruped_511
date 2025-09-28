/*
 * @Author: Si
 * @Date: 2024-06-27 19:16:52
 * @LastEditTime: 2024-10-09 22:29:41
 * @LastEditors: Please set LastEditors
 * @Description: 任务4，摆动腿任务（一条腿）
 * @FilePath: /robot_software_1/user/leg_controller/WBC_Ctrl/TaskSet/LinkPosTask.cpp
 */

#include "LinkPosTask.h"
// (X, Y, Z)


// #define TK Task<T>

/**
 * @brief 任务4，摆动腿任务，构造函数
 * @tparam T 
 * @param robot 
 * @param link_idx 
 */
template <typename T>
LinkPosTask<T>::LinkPosTask(const FBModelState<T>* fb_state, int leg_idx)
        :Task<T>(3),
         _fb_state(fb_state),
         _leg_idx(leg_idx)
{
    Task<T>::Jt_ = DMat<T>::Zero(Task<T>::dim_task_, (size_t)18);
    Task<T>::JtDotQdot_ = DVec<T>::Zero(Task<T>::dim_task_);

    _Kp_kin = DVec<T>::Constant(Task<T>::dim_task_, 1.);
    _Kp = DVec<T>::Constant(Task<T>::dim_task_, 100.);
    _Kd = DVec<T>::Constant(Task<T>::dim_task_, 5.);
}


/**
 * @description: 更新xddot_cmd的函数，没看懂。
 *  输入：
 * @param pos_des 四元数表示的desire角度
 * @param vel_des desire角速度
 * @param acc_des desire角加速度，猜测这一项为0
 *  输出：
 * @param op_cmd_ 参与加速度WBC计算的xddot_cmd
 * @return {*}
 */
template <typename T>
bool LinkPosTask<T>::_UpdateCommand(const void* pos_des, const DVec<T>& vel_des, const DVec<T>& acc_des){
    Vec3<T>* pos_cmd =  (Vec3<T>*)pos_des;
    Vec3<T> link_pos;

    link_pos = _fb_state->p.block(_leg_idx * 3, 0, 3, 1); // foot position in world

    // X, Y, Z
    for(int i(0); i < 3; ++i){
        Task<T>::pos_err_[i] = _Kp_kin[i] * ( (*pos_cmd)[i] - link_pos[i] );
        Task<T>::vel_des_[i] = vel_des[i];
        Task<T>::acc_des_[i] = acc_des[i];
    }

    //更新op_cmd_
    for(size_t i(0); i < Task<T>::dim_task_; ++i){
        Vec3<T> block = _fb_state->v.block(_leg_idx * 3, 0, 3, 1);

        Task<T>::op_cmd_[i] =   _Kp[i] * Task<T>::pos_err_[i] 
                              + _Kd[i] * (Task<T>::vel_des_[i] - block(i))
                              + Task<T>::acc_des_[i];
    }

    return true;
}

/**
 * @brief 更新任务的雅可比矩阵Jt_，从顶层调用LocomotionCtrl获取摆动腿足端的雅可比矩阵
 * @tparam T 
 * @return true 
 * @return false 
 */
template <typename T>
bool LinkPosTask<T>::_UpdateFootTaskJacobian(Mat3_18<float> Jc) {
  Task<T>::Jt_ = Jc;

  // if (!virtual_depend_) { //关
  //   Task<T>::Jt_.block(0, 0, 3, 6) = DMat<T>::Zero(3, 6);
  // }
  
  return true;
}

/**
 * @brief 更新任务的JDotQdot，从顶层调用LocomotionCtrl获取摆动腿足端的JDotQdot
 * @tparam T 
 * @return true 
 * @return false 
 */
template <typename T>
bool LinkPosTask<T>::_UpdateFootTaskJDotQdot(Vec3<float> JcdotQdot) {
  Task<T>::JtDotQdot_ = JcdotQdot;
  return true;
}


template class LinkPosTask<float>;

