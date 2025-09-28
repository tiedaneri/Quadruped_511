/*
 * @Author: Si
 * @Date: 2024-06-25 19:47:54
 * @LastEditTime: 2024-10-08 22:51:28
 * @LastEditors: Please set LastEditors
 * @Description: 定义了任务基类用于任务的描述与构建，其中定义了任务矩阵维度，Jacobian更新方法，
                 期望更新方法，在WBC_Ctrl/TaskSet文件夹下通过继承该基类定义了Base位置，姿态等任务。
 * @FilePath: /robot_software_1/user/leg_controller/WBC_Ctrl/Task.h
 */

#ifndef WBC_TASK
#define WBC_TASK

#include <cppTypes.h>

// #define TK Task<T>

template <typename T>
class Task{
public:
    Task(size_t dim)
    :   b_set_task_(false),
        dim_task_(dim),
        op_cmd_(dim),
        pos_err_(dim),
        vel_des_(dim),
        acc_des_(dim) 
    {}

    virtual ~Task(){}

    void getCommand(DVec<T>& op_cmd) { op_cmd = op_cmd_ ;} //获取任务的加速度规划
    void getTaskJacobian(DMat<T>& Jt) { Jt = Jt_; }
    void getTaskJacobianDotQdot(DVec<T>& JtDotQdot) { JtDotQdot = JtDotQdot_ ;}

	//机身任务
    bool UpdateTask(const void* pos_des, const DVec<T>& vel_des, const DVec<T>& acc_des) {
		_UpdateTaskJacobian();
		_UpdateTaskJDotQdot();
		_UpdateCommand(pos_des, vel_des, acc_des);
		_AdditionalUpdate();
		b_set_task_ = true;
		return true;
  	}

	//足底摆动任务
	bool UpdateFootTask(Mat3_18<float> Jc, Vec3<float> JcdotQdot, const void* pos_des, const DVec<T>& vel_des, const DVec<T>& acc_des) {
		_UpdateFootTaskJacobian(Jc);
		_UpdateFootTaskJDotQdot(JcdotQdot);
		_UpdateCommand(pos_des, vel_des, acc_des);
		_AdditionalUpdate();
		b_set_task_ = true;
		return true;
  	}

    bool IsTaskSet() { return b_set_task_; }
	size_t getDim() { return dim_task_; }
	void UnsetTask() { b_set_task_ = false; }

	const DVec<T>& getPosError() { return pos_err_; }
	const DVec<T>& getDesVel() { return vel_des_; }
	const DVec<T>& getDesAcc() { return acc_des_; }

protected:
    // 更新任务的加速度规划
	virtual bool _UpdateCommand(const void* pos_des, const DVec<T>& vel_des,
								const DVec<T>& acc_des) = 0;
	// Update Jt_
	virtual bool _UpdateTaskJacobian() = 0;
	// Update JtDotQdot_
	virtual bool _UpdateTaskJDotQdot() = 0;

	// Update Foot Task Jt_
	virtual bool _UpdateFootTaskJacobian(Mat3_18<float> Jc) = 0;
	// Update Foot Task JtDotQdot_
	virtual bool _UpdateFootTaskJDotQdot(Vec3<float> JcdotQdot) = 0;

	// Additional Update (defined in child classes)
	virtual bool _AdditionalUpdate() = 0;

    bool b_set_task_;
	size_t dim_task_;

	DVec<T> op_cmd_;
    DMat<T> Jt_; //任务雅各比
	DVec<T> JtDotQdot_; //任务JtDotQdot
	

	DVec<T> pos_err_;
	DVec<T> vel_des_;
	DVec<T> acc_des_;
};


// template <typename T>
// struct FBModelState { //WBC需要的浮动基模型状态
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//     Quat<T> bodyOrientation; //世界坐标系下，机身姿态四元数
//     Vec3<T> bodyPosition;    //世界坐标系下，机身位置
//     SVec<T> bodyVelocity;    //机身坐标系下，六维广义速度 

//     DVec<T> q;               //12维，关节位置
//     DVec<T> qd;              //12维，关节速度
// };

#endif /*WBC_TASK*/
