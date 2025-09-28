/*
 * @Author: your name
 * @Date: 2024-06-27 19:11:45
 * @LastEditTime: 2024-10-09 15:00:40
 * @LastEditors: Please set LastEditors
 * @Description: 任务3，机体位置任务
 * @FilePath: /robot_software_1/user/leg_controller/WBC_Ctrl/TaskSet/BodyPosTask.h
 */
#ifndef BODY_POS_TASK
#define BODY_POS_TASK

#include "WBC_Ctrl/Task.h"
#include "WBC_Ctrl.h"


template <typename T>
class BodyPosTask : public Task<T>{
public:
    BodyPosTask(const FBModelState<T>* fb_state);
    ~BodyPosTask() {}

    const FBModelState<T>* _fb_state;

    DVec<T> _Kp_kin;
    DVec<T> _Kp, _Kd;

protected:
    // Update op_cmd_
    virtual bool _UpdateCommand(const void* pos_des, const DVec<T>& vel_des, const DVec<T>& acc_des);
    // Update Jt_
    virtual bool _UpdateTaskJacobian();
    // Update JtDotQdot_
    virtual bool _UpdateTaskJDotQdot() { return true; }
    virtual bool _AdditionalUpdate()   { return true; }

    // 因为Task类有纯虚函数不能实现，确保过编译器
	virtual bool _UpdateFootTaskJacobian(Mat3_18<float> Jc) { return true; }
	virtual bool _UpdateFootTaskJDotQdot(Vec3<float> JcdotQdot){ return true; }

};



#endif /*BODY_POS_TASK*/