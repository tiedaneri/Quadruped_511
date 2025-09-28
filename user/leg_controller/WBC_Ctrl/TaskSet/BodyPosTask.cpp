/*
 * @Author: your name
 * @Date: 2024-06-27 19:11:34
 * @LastEditTime: 2024-10-09 22:38:38
 * @LastEditors: Please set LastEditors
 * @Description: 任务3，机体位置任务
 * @FilePath: /robot_software_1/user/leg_controller/WBC_Ctrl/TaskSet/BodyPosTask.cpp
 */
#include "BodyPosTask.h"
// (X, Y, Z)
#include "Math/orientation_tools.h"



// #define TK Task<T>

/**
 * @brief 任务3构造函数
 * @tparam T 
 * @param robot 
 */
template <typename T>
BodyPosTask<T>::BodyPosTask(const FBModelState<T>* fb_state)
        :Task<T>(3), _fb_state(fb_state)
{
    Task<T>::Jt_ = DMat<T>::Zero(Task<T>::dim_task_, (size_t)18);
    Task<T>::Jt_.block(0, 3, 3, 3).setIdentity();
	Task<T>::JtDotQdot_ = DVec<T>::Zero(Task<T>::dim_task_);

    _Kp_kin = DVec<T>::Constant(Task<T>::dim_task_, 1.);
	_Kp = DVec<T>::Constant(Task<T>::dim_task_, 50.);
	_Kd = DVec<T>::Constant(Task<T>::dim_task_, 1.0);
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
bool BodyPosTask<T>::_UpdateCommand(const void* pos_des, const DVec<T>& vel_des, const DVec<T>& acc_des){
    Vec3<T>* pos_cmd = (Vec3<T>*)pos_des;
    // Vec3<T> link_pos = _robot_sys->_state.bodyPosition; //在顶层调用中（_UpdateModel函数）更新了机身位置
    Vec3<T> link_pos = _fb_state->bodyPosition; //在顶层调用中（_UpdateModel函数）更新了机身位置


    
    // Quat<T> quat = _robot_sys->_state.bodyOrientation; //在顶层调用中（_UpdateModel函数）更新了机身位姿
    Quat<T> quat = _fb_state->bodyOrientation; //在顶层调用中（_UpdateModel函数）更新了机身位姿
    Mat3<T> Rot = ori::quaternionToRotationMatrix(quat); //拿到机身相对于世界系的旋转矩阵

    
    // SVec<T> curr_vel = _robot_sys->_state.bodyVelocity; //在顶层调用中（_UpdateModel函数）更新了广义速度
    SVec<T> curr_vel = _fb_state->bodyVelocity; //在顶层调用中（_UpdateModel函数）更新了广义速度
    curr_vel.tail(3) = Rot.transpose() * curr_vel.tail(3); //实际速度转到世界坐标系下

    // X, Y, Z
    for(int i(0); i < 3; ++i){
        Task<T>::pos_err_[i] = _Kp_kin[i] * ((*pos_cmd)[i] - link_pos[i]);
        Task<T>::vel_des_[i] = vel_des[i];
		Task<T>::acc_des_[i] = acc_des[i];
        
        // 对应MIT论文中的公式(22)
        Task<T>::op_cmd_[i] = _Kp[i] * ((*pos_cmd)[i] - link_pos[i]) + _Kd[i] * (Task<T>::vel_des_[i] - curr_vel[i + 3]) + Task<T>::acc_des_[i]; 
    }
    return true;
}

/**
 * @brief 更新任务的雅可比矩阵Jt_，对应yxy论文P55最上面（4.40），
 *        旋转矩阵是本体系到世界坐标系的映射。
 * @tparam T 
 * @return true 
 */
template <typename T>
bool BodyPosTask<T>::_UpdateTaskJacobian(){
    // Quat<T> quat = _robot_sys->_state.bodyOrientation; //在顶层调用中（_UpdateModel函数）更新了机身位姿
    Quat<T> quat = _fb_state->bodyOrientation; //在顶层调用中（_UpdateModel函数）更新了机身位姿
	Mat3<T> Rot = ori::quaternionToRotationMatrix(quat); //旋转矩阵
    
	Task<T>::Jt_.block(0, 3, 3, 3) = Rot.transpose();
	// TK::Jt_.block(0,3, 3,3) = Rot;
	// pretty_print(TK::Jt_, std::cout, "Jt");
	// TK::Jt_.block(0,3, 3,3) = Rot*TK::Jt_.block(0,3,3,3);
  	return true;
}


template class BodyPosTask<float>;