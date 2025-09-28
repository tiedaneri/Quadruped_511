/*
 * @Author: your name
 * @Date: 2024-06-27 18:59:53
 * @LastEditTime: 2024-10-09 15:00:29
 * @LastEditors: Please set LastEditors
 * @Description: 任务2，机体姿态任务.h
 * @FilePath: /robot_software_1/user/leg_controller/WBC_Ctrl/TaskSet/BodyOriTask.h
 */
#ifndef BODY_ORIENTATION_TASK
#define BODY_ORIENTATION_TASK

#include "WBC_Ctrl/Task.h"
#include "WBC_Ctrl.h"


template <typename T>
class BodyOriTask : public Task<T> {
public:
    BodyOriTask(const FBModelState<T>* fb_state);
    ~BodyOriTask() {} 
    
    const FBModelState<T>* _fb_state;

    DVec<T> _Kp_kin;
    DVec<T> _Kp, _Kd;

protected:
    // Update op_cmd_(xddot_cmd)
	virtual bool _UpdateCommand(const void* pos_des, const DVec<T>& vel_des, const DVec<T>& acc_des);
    // Update Jt_
	virtual bool _UpdateTaskJacobian();
	// Update JtDotQdot_ 任务2和3的JDotQdot_均为0阵（3x1）。
	virtual bool _UpdateTaskJDotQdot() { return true; }

    virtual bool _AdditionalUpdate() { return true; }

    // 因为Task类有纯虚函数不能实现，确保过编译器
	virtual bool _UpdateFootTaskJacobian(Mat3_18<float> Jc) { return true; }
	virtual bool _UpdateFootTaskJDotQdot(Vec3<float> JcdotQdot){ return true; }
};

#endif /*BODY_ORIENTATION_TASK*/
