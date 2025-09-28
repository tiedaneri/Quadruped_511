/*
 * @Author: Si
 * @Date: 2024-07-01 10:03:19
 * @LastEditTime: 2024-10-23 10:05:21
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /robot_software_1/user/leg_controller/WBC_Ctrl/LocomotionCtrl.cpp
 */
#include "LocomotionCtrl.h"
#include "WBC_Ctrl/ContactSet/SingleContact.h"
#include "WBC_Ctrl/TaskSet/BodyOriTask.h"
#include "WBC_Ctrl/TaskSet/BodyPosTask.h"
#include "WBC_Ctrl/TaskSet/LinkPosTask.h"

#include "Math/orientation_tools.h"

// #define WBCtrl WBC_Ctrl<T>

/**
 * @brief 构造函数
 * @param 
 */
template<typename T>
LocomotionCtrl<T>::LocomotionCtrl(fb_dynamic* dynamic, body_state_estimator_data* body_data, leg_state_data* leg_state)
        :WBC_Ctrl<T>(dynamic, body_data, leg_state)
{
    _foot_contact[0] = new SingleContact<T>();
    _foot_contact[1] = new SingleContact<T>();
    _foot_contact[2] = new SingleContact<T>();
    _foot_contact[3] = new SingleContact<T>();

    _body_ori_task = new BodyOriTask<T>(&(WBCtrl::fbm_state));
    _body_pos_task = new BodyPosTask<T>(&(WBCtrl::fbm_state));

    _foot_task[0] = new LinkPosTask<T>(&(WBCtrl::fbm_state), 0);
    _foot_task[1] = new LinkPosTask<T>(&(WBCtrl::fbm_state), 1);
    _foot_task[2] = new LinkPosTask<T>(&(WBCtrl::fbm_state), 2);
    _foot_task[3] = new LinkPosTask<T>(&(WBCtrl::fbm_state), 3);
}

/**
 * @brief 析构函数
 */
template<typename T>
LocomotionCtrl<T>::~LocomotionCtrl(){
    delete _body_ori_task;
    delete _body_pos_task;

    for(size_t i(0); i < 4; ++i){
        delete _foot_contact[i];
        delete _foot_task[i];
    }
}

/**
 * @brief 更新四个任务
 * @param input WBC顶层调用WBC_run()的输入，定义在Locomotion头文件
 */
template<typename T>
void LocomotionCtrl<T>::_ContactTaskUpdate(void* input){
    _input_data = static_cast<LocomotionCtrlData<T>* >(input); //强制类型转换成LocomotionCtrlData*

    _ParameterSetup(); //调用_ParameterSetup函数完成所有Kp、Kd参数的初始化
	
    _CleanUp(); //调用_CleanUp清空接触列表与任务列表

    _quat_des = ori::rpyToQuat(_input_data->pBody_RPY_des);//rpy转四元数，作为机身旋转任务的pos_des

    Vec3<T> zero_vec3; 
    zero_vec3.setZero(); // acc_des，均为0

    _body_ori_task->UpdateTask(&_quat_des, _input_data->vBody_Ori_des, zero_vec3); //激活机体姿态任务2
    _body_pos_task->UpdateTask(&(_input_data->pBody_des), _input_data->vBody_des, _input_data->aBody_des);//激活机体位置任务3
    WBCtrl::_task_list.push_back(_body_ori_task);
	WBCtrl::_task_list.push_back(_body_pos_task);

    for(size_t leg(0); leg < 4; ++leg){
        if(_input_data->contact_state[leg] > 0.){ //支撑腿
            _foot_contact[leg]->setRFDesired((DVec<T>)(_input_data->Fr_des[leg])); //从输入获取地面反作用力
            _foot_contact[leg]->UpdateContactSpec(WBC_Ctrl<T>::Jc[leg], WBC_Ctrl<T>::JcdotQdot[leg]); //激活任务1
            WBCtrl::_contact_list.push_back(_foot_contact[leg]);
        }
        else{ //摆动腿
            _foot_task[leg]->UpdateFootTask(WBC_Ctrl<T>::Jc[leg], WBC_Ctrl<T>::JcdotQdot[leg], &(_input_data->pFoot_des[leg]), _input_data->vFoot_des[leg], _input_data->aFoot_des[leg]);//激活任务4
            WBCtrl::_task_list.push_back(_foot_task[leg]);
        }
    }
}


/**
 * @brief 设置Kp、Kd参数（double类型），计算相关任务的xddot_cmd的参数
 *        这里接触腿没有相关系数，是因为接触任务的xddot_cmd确定为0了
 * @param param 
 */
template<typename T>
void LocomotionCtrl<T>::_ParameterSetup(){
    for(size_t i(0); i<3; ++i){
        ((BodyOriTask<T>*)_body_ori_task)->_Kp[i] = 100.;
        ((BodyOriTask<T>*)_body_ori_task)->_Kd[i] = 10.;

        ((BodyPosTask<T>*)_body_pos_task)->_Kp[i] = 10.;
        ((BodyPosTask<T>*)_body_pos_task)->_Kd[i] = 3.;

        for(size_t j(0); j<4; ++j){
            ((LinkPosTask<T>*)_foot_task[j])->_Kp[i] = 70.;
            ((LinkPosTask<T>*)_foot_task[j])->_Kd[i] = 3.;
        }

        // WBCtrl::_Kp_joint[i] = Kp_joint[i]; //发给电机的kp----------------------------------------------------
        // WBCtrl::_Kd_joint[i] = Kd_joint[i]; //发给电机的kd
    }

    WBCtrl::_Kp_joint = {3, 3, 3};
    WBCtrl::_Kd_joint = {1, 0.2, 0.2};
}

/**
 * @brief 清空接触列表与任务列表
 */
template<typename T>
void LocomotionCtrl<T>::_CleanUp(){
    WBCtrl::_contact_list.clear();
    WBCtrl::_task_list.clear();
}

template class LocomotionCtrl<float>;