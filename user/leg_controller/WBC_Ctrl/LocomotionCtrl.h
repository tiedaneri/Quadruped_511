/*
 * @Author: Si
 * @Date: 2024-07-01 09:08:15
 * @LastEditTime: 2024-10-22 15:41:01
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /robot_software_1/user/leg_controller/WBC_Ctrl/LocomotionCtrl.h
 */
#ifndef LOCOMOTION_CONTROLLER
#define LOCOMOTION_CONTROLLER

#include "WBC_Ctrl/WBC_Ctrl.h"

template<typename T>
class LocomotionCtrlData{ //WBC最顶层模块入口函数WBC_run()的第一个参数input
public:
    Vec3<T> pBody_des;
    Vec3<T> vBody_des;
    Vec3<T> aBody_des;
    Vec3<T> pBody_RPY_des;
    Vec3<T> vBody_Ori_des;

    Vec3<T> pFoot_des[4];
    Vec3<T> vFoot_des[4];
    Vec3<T> aFoot_des[4];
    Vec3<T> Fr_des[4];

    Vec4<T> contact_state; //是否触地，１为触地，０为摆动
};

template<typename T>
class LocomotionCtrl : public WBC_Ctrl<T>{
public:
    LocomotionCtrl(fb_dynamic* dynamic, body_state_estimator_data* body_data, leg_state_data* leg_state);
    virtual ~LocomotionCtrl();

protected:
    virtual void _ContactTaskUpdate(void* input);
    void _ParameterSetup();
    void _CleanUp();

    LocomotionCtrlData<T>* _input_data;

    ContactSpec<T>* _foot_contact[4];
    Task<T>* _body_ori_task;
    Task<T>* _body_pos_task;
    Task<T>* _foot_task[4];
    
    Vec3<T> pre_foot_vel[4];

    Vec3<T> _Fr_result[4];
    Quat<T> _quat_des; //pBody_RPY_des的四元数表示


};


#endif /*LOCOMOTION_CONTROLLER*/