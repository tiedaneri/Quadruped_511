/*
 * @Author: Si
 * @Date: 2024-06-26 00:21:02
 * @LastEditTime: 2024-10-09 22:49:50
 * @LastEditors: Please set LastEditors
 * @Description: （针对单个脚的）接触任务，！！！！！！！！！！！对于Jc和JcDotQdot的计算位于浮基动力学中！！！！！！！！！！！
 * @FilePath: /robot_software_1/user/leg_controller/WBC_Ctrl/ContactSet/SingleContact.cpp
 */
#include "SingleContact.h"

// [ Fx, Fy, Fz ]

/**
 * @description: 构造函数
 * 
 * @param {ContactSpec<T>} 接触基类，初始化为3，即接触维度是3
 * @param {_max_Fz} 摩擦锥中，最大Fz
 * @return {*}
 */
template <typename T>
SingleContact<T>::SingleContact()
        :ContactSpec<T>(3), _max_Fz(1500.), _dim_U(6)
{
    ContactSpec<T>::idx_Fz_ = 2;
    ContactSpec<T>::Jc_ = DMat<T>::Zero(ContactSpec<T>::dim_contact_, size_t(18));
    ContactSpec<T>::JcDotQdot_ = DVec<T>::Zero(ContactSpec<T>::dim_contact_);
    ContactSpec<T>::Uf_ = DMat<T>::Zero(_dim_U, ContactSpec<T>::dim_contact_); //??????????????????????????

    T mu(0.4);

    ContactSpec<T>::Uf_(0, 2) = 1.;
    
    ContactSpec<T>::Uf_(1, 0) = 1.;
    ContactSpec<T>::Uf_(1, 2) = mu;
    ContactSpec<T>::Uf_(2, 0) = -1.;
    ContactSpec<T>::Uf_(2, 2) = mu;

    ContactSpec<T>::Uf_(3, 1) = 1.;
    ContactSpec<T>::Uf_(3, 2) = mu;
    ContactSpec<T>::Uf_(4, 1) = -1.;
    ContactSpec<T>::Uf_(4, 2) = mu;

    // Upper bound of normal force
    ContactSpec<T>::Uf_(5, 2) = -1.;
}

/**
 * @description: 更新这个接触脚的Jc
 * @param {*}
 * @return {*}
 */
template <typename T>
bool SingleContact<T>::_UpdateJc(Mat3_18<float> Jc){
    ContactSpec<T>::Jc_ = Jc;
    return true;
}

/**
 * @description: 更新这个接触脚的JcDotQdot
 * @param {*}
 * @return {*}
 */
template <typename T>
bool SingleContact<T>::_UpdateJcDotQdot(Vec3<float> JcdotQdot) {
	ContactSpec<T>::JcDotQdot_ = JcdotQdot;
	// pretty_print(ContactSpec<T>::JcDotQdot_, std::cout, "JcDotQdot");
	return true;
}

/**
 * @description: 更新这个接触脚的？？？，用于摩擦锥中不等式最小值阵；其实也不叫更新，这跟Uf_一样也是个常值矩阵
 * @param {*}
 * @return {*}
 */
template <typename T>
bool SingleContact<T>::_UpdateInequalityVector() {
	ContactSpec<T>::ieq_vec_ = DVec<T>::Zero(_dim_U); //6行向量
	ContactSpec<T>::ieq_vec_[5] = -_max_Fz; //第5行
	return true;
}

template class SingleContact<float>;
