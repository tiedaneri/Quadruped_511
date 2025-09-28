/*
 * @Author: Si
 * @Date: 2024-07-01 09:13:59
 * @LastEditTime: 2024-10-23 10:04:34
 * @LastEditors: Please set LastEditors
 * @Description: WBC模块的集成调用
 * @FilePath: /robot_software_1/user/leg_controller/WBC_Ctrl/WBC_Ctrl.h
 */
#ifndef WBC_CONTROLLER
#define WBC_CONTROLLER

#include "cppTypes.h"
#include "WBC_Ctrl/KinWBC.h"
#include "WBC_Ctrl/WBIC.h"
#include "body_state_estimator.h"  //机身状态估计
#include "leg_driver.h"
#include "fb_dynamic.h"//动力学，包含动力学矩阵、任务雅、JdotQdot


#define WBCtrl WBC_Ctrl<T>

class MIT_UserParameters;

//WBC需要的浮动基模型状态
template <typename T>
struct FBModelState { 
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Quat<T> bodyOrientation; //世界坐标系下，机身姿态四元数
    Vec3<T> bodyPosition;    //世界坐标系下，机身位置
    SVec<T> bodyVelocity;    //机身坐标系下，六维广义速度 

    Vec12<T> q;               //12维，关节位置
    Vec12<T> qd;              //12维，关节速度

    Vec12<float> p, v;        //12维，机身坐标系下，四个足端位置、速度
};

//WBC输出
struct WBC_legcmd
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Vec3<float> q_des, qd_des, tau_ff;  //期望位置、期望速度、前馈力矩
    Mat3<float> kp, kd;                 //比例控制系数、微分控制系数
};

/*
    
*/
template<typename T>
class WBC_Ctrl{
public:
    //在类中定义Eigen变量时要加入下面这句用于数据对齐
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //指针变量，动力学，在leg_controller中已经完成计算；真实对象在leg_controller中
    fb_dynamic* _dynamic;
    //指针变量，存储估计的身体状态结果；真实对象在robot_console中
    body_state_estimator_data* _body_data;
    //指针变量，指向储存四个腿数据的结构体数组；真实对象在robot_console的leg_driver中
    leg_state_data* _l_state;

    WBC_legcmd _WBC_legcmd[4]; //WBC输出结构体-----------------------------

    WBC_Ctrl(fb_dynamic* dynamic, body_state_estimator_data* body_data, leg_state_data* l_state);
    virtual ~WBC_Ctrl();

    void WBC_run(void* input); 

    void setFloatingBaseWeight(const T & weight){ //修改松弛优化权重阵Q1外部接口函数
        _wbic_data->_W_floating = DVec<T>::Constant(6, weight); //松弛优化权重阵Q1，在WBC_Ctrl构造函数中初始化
    }

protected:
    void _UpdateModel(fb_dynamic* dynamic, body_state_estimator_data* body_data, leg_state_data* l_state);
    virtual void _ContactTaskUpdate(void * input) = 0; //在LocomotionCtrl中定义
    void _ComputeWBC();
    void _UpdateLegCMD();
    
    KinWBC<T>* _kin_wbc;
    WBIC<T>* _wbic;
    WBIC_ExtraData<T>* _wbic_data;
    
    std::vector<ContactSpec<T>*> _contact_list;
    std::vector<Task<T>*> _task_list;
    
    Mat3_18<float> Jc[4];
    Vec3<float> JcdotQdot[4];

    DMat<T> _A;
	DMat<T> _Ainv;	
	DVec<T> _C;

    //腿的基坐标系相对于机身坐标系的位置
    const Vec3<float> _hip_location[4] = 
    {
        Vec3<float>( 0.196, -0.05, 0),
        Vec3<float>( 0.196,  0.05, 0),
        Vec3<float>(-0.196, -0.05, 0),
        Vec3<float>(-0.196,  0.05, 0)
    };

    FBModelState<T> fbm_state; //在_UpdateModel()执行更新，储存更新所需信息的结构体
    
    DVec<T> _full_config;
	DVec<T> _tau_ff; //12维向量,电机的前馈力矩
	DVec<T> _des_jpos;
	DVec<T> _des_jvel; //计算结果

    std::vector<T> _Kp_joint, _Kd_joint; //发给电机的kp、kd

    unsigned long long _iter;
};

#endif /*WBC_CONTROLLER*/
