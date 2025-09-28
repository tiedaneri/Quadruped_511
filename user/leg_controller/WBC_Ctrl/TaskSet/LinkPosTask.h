/*
 * @Author: Si
 * @Date: 2024-06-27 19:16:42
 * @LastEditTime: 2024-10-09 14:59:27
 * @LastEditors: Please set LastEditors
 * @Description: 任务4，摆动腿任务（一条腿）
 * @FilePath: /robot_software_1/user/leg_controller/WBC_Ctrl/TaskSet/LinkPosTask.h
 */
#ifndef LINK_POS_TASK
#define LINK_POS_TASK

// (X, Y, Z)
#include "WBC_Ctrl/Task.h"
#include "WBC_Ctrl.h"

template <typename T>
class LinkPosTask : public Task<T>{
public:
    LinkPosTask(const FBModelState<T>* fb_state, int leg_idx);
    ~LinkPosTask() {}

    const FBModelState<T>* _fb_state;

    int _leg_idx;

    DVec<T> _Kp_kin;
    DVec<T> _Kp, _Kd;
    
protected:
    // Update op_cmd_
    virtual bool _UpdateCommand(const void* pos_des, const DVec<T>& vel_des, const DVec<T>& acc_des);
    // Update Jt_
    virtual bool _UpdateFootTaskJacobian(Mat3_18<float> Jc);
    // Update JtDotQdot_
    virtual bool _UpdateFootTaskJDotQdot(Vec3<float> JcdotQdot);
    virtual bool _AdditionalUpdate() { return true; }

    // 因为Task类有纯虚函数不能实现，确保过编译器
	virtual bool _UpdateTaskJacobian() {return true;}
	virtual bool _UpdateTaskJDotQdot() {return true;}

    
};



#endif /*LINK_POS_TASK*/