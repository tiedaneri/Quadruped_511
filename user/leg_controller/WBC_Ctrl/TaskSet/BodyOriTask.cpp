/*
 * @Author: your name
 * @Date: 2024-06-27 18:57:29
 * @LastEditTime: 2024-10-09 11:25:01
 * @LastEditors: Please set LastEditors
 * @Description: 任务2， 机体姿态任务.cpp
 * @FilePath: /robot_software_1/user/leg_controller/WBC_Ctrl/TaskSet/BodyOriTask.cpp
 */
#include "BodyOriTask.h"

// (Rx, Ry, Rz)

#include "Math/orientation_tools.h"


template <typename T>
BodyOriTask<T>::BodyOriTask(const FBModelState<T>* fb_state)
        :Task<T>(3), _fb_state(fb_state)
{
    Task<T>::Jt_ = DMat<T>::Zero(Task<T>::dim_task_, (size_t)18);
    Task<T>::Jt_.block(0, 0, 3, 3).setIdentity();
    Task<T>::JtDotQdot_ = DVec<T>::Zero(Task<T>::dim_task_);

    _Kp_kin = DVec<T>::Constant(Task<T>::dim_task_, 1.);
    _Kp = DVec<T>::Constant(Task<T>::dim_task_, 50.);
	_Kd = DVec<T>::Constant(Task<T>::dim_task_, 1.);
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
bool BodyOriTask<T>::_UpdateCommand(const void* pos_des, const DVec<T>& vel_des, const DVec<T>& acc_des){
    Quat<T>* ori_cmd = (Quat<T>*)pos_des; //目标角度，给的是四元数
    // Quat<T> link_ori = (_robot_sys->_state.bodyOrientation); //在顶层调用中（_UpdateModel函数）更新了机身角度
    Quat<T> link_ori = (_fb_state->bodyOrientation); //在顶层调用中（_UpdateModel函数）更新了机身角度

    Quat<T> link_ori_inv;
	link_ori_inv[0] = link_ori[0];
	link_ori_inv[1] = -link_ori[1];
	link_ori_inv[2] = -link_ori[2];
	link_ori_inv[3] = -link_ori[3];

    Quat<T> ori_err = ori::quatProduct(*ori_cmd, link_ori_inv); // 四元数相乘代表连续旋转
    if (ori_err[0] < 0.) ori_err *= (-1.); // 四元数第一个数必须大于0

    Vec3<T> ori_err_so3;
    ori::quaternionToso3(ori_err, ori_err_so3);// 四元数转旋转向量

    // SVec<T> curr_vel = _robot_sys->_state.bodyVelocity; //在顶层调用中（_UpdateModel函数）更新了机身广义速度(6维)
    SVec<T> curr_vel = _fb_state->bodyVelocity; //在顶层调用中（_UpdateModel函数）更新了机身广义速度(6维)
    // 将角速度 变换到与 世界系下角速度相同的基下来描述
	Mat3<T> Rot = ori::quaternionToRotationMatrix(link_ori);
	Vec3<T> vel_err = Rot.transpose()*(Task<T>::vel_des_ - curr_vel.head(3));

    for (int i(0); i < 3; ++i) {
		Task<T>::pos_err_[i] = _Kp_kin[i] * ori_err_so3[i];
		Task<T>::vel_des_[i] = vel_des[i];
		Task<T>::acc_des_[i] = acc_des[i];

		Task<T>::op_cmd_[i] = _Kp[i] * ori_err_so3[i] + _Kd[i] * vel_err[i] + Task<T>::acc_des_[i];
	}

    return true;
}

/**
 * @brief 更新任务的雅可比矩阵Jt_,对应yxy论文P54最下面（4.39）
 * @tparam T 
 * @return true 
 */
template <typename T>
bool  BodyOriTask<T>::_UpdateTaskJacobian(){
    // Quat<T> quat = _robot_sys->_state.bodyOrientation;
    Quat<T> quat = _fb_state->bodyOrientation;
    Mat3<T> Rot = ori::quaternionToRotationMatrix(quat);

    Task<T>::Jt_.block(0, 0, 3, 3) = Rot.transpose();
    //pretty_print(Rot, std::cout, "Rot mat");
    return true;
}


template class BodyOriTask<float>;
