/*
 * @Author: Si
 * @Date: 2024-07-01 09:13:51
 * @LastEditTime: 2025-01-05 23:39:07
 * @LastEditors: Please set LastEditors
 * @Description: WBC模块的集成调用
 * @FilePath: /robot_software_1/user/leg_controller/WBC_Ctrl/WBC_Ctrl.cpp
 */
#include "WBC_Ctrl/WBC_Ctrl.h"


/**
 * @description: 构造函数
 * @param {model}
 * @return {*}
 */
template<typename T>
WBC_Ctrl<T>::WBC_Ctrl(fb_dynamic* dynamic, body_state_estimator_data* body_data, leg_state_data* leg_state)
        :_full_config((size_t)(12 + 7)),
         _tau_ff((size_t)12),
	     _des_jpos((size_t)12),
	     _des_jvel((size_t)12)
{
    _iter = 0;
    _full_config.setZero(); // 19行Vec

    _dynamic = dynamic;     //连接动力学计算
    _body_data = body_data; //连接状态估计器数据结构体
    _l_state = leg_state;     //连接腿数据结构体
    
    _kin_wbc = new KinWBC<T>((size_t)18);

    _wbic = new WBIC<T>((size_t)18, &(_contact_list), &(_task_list));
    _wbic_data = new WBIC_ExtraData<T>();

    //WBIC中松弛优化的权重矩阵
    _wbic_data->_W_floating = DVec<T>::Constant(6, 0.1); //Q1,全初始化为0.1，setFloatingBaseWeight()函数还会修改
    _wbic_data->_W_rf = DVec<T>::Constant(12, 0.9);//Q2,全初始化为0.1
    
    _Kp_joint.resize((size_t)3, 5.);
	_Kd_joint.resize((size_t)3, 1.5);
}


/**
 * @description: 析构函数
 */
template<typename T>
WBC_Ctrl<T>::~WBC_Ctrl(){
    delete _kin_wbc;
    delete _wbic;
    delete _wbic_data;

    typename std::vector<Task<T>*>::iterator iter = _task_list.begin();
    while(iter < _task_list.end()){
        delete (*iter);
        ++iter;
    }
    _task_list.clear();

    typename std::vector<ContactSpec<T> *>::iterator iter2 = _contact_list.begin();
    while (iter2 < _contact_list.end()) {
        delete (*iter2);
        ++iter2;
    }
    _contact_list.clear();
}


/**
 * @description: WBC功能模块的最顶层入口
 * @param {input} 为LocomotionCtrlData类型，WBC模块的输入
 * @return {*}
 */
template<typename T>
void WBC_Ctrl<T>::WBC_run(void* input){
    ++_iter;
    
    // 更新WBC计算中用到的模型参数, 使用动力学、状态估计、腿控制器
	_UpdateModel(_dynamic, _body_data, _l_state); 

    // 任务以及接触相关的更新（位于LocomotionCtrl）
	_ContactTaskUpdate(input);

    // WBC计算
	_ComputeWBC();

    // 更新腿部指令
	_UpdateLegCMD();
}


/**
 * @brief 更新WBC计算过程中使用到的模型参数:
 * 从状态估计器拿到位置和姿态、机身广义速度（6维），从腿数据拿到12关节的q和qd，调用动力学拿接触雅和JcdotQdot，从原有动力学取得H与C项
 * @param dynamic 动力学计算指针
 * @param body_data 状态估计器数据结构体指针
 * @param l_state 腿数据结构体指针：关节位置、关节速度、机身位置、机身速度
 */
template<typename T>
void WBC_Ctrl<T>::_UpdateModel(fb_dynamic* dynamic, body_state_estimator_data* body_data, leg_state_data* l_state){
    fbm_state.bodyOrientation = body_data->orientation;  //姿态四元数
	fbm_state.bodyPosition = body_data->position;        //位置三维向量

    Vec3<T> tmp_p[4],tmp_v[4];
    //更新腿基坐标系下的p和v到世界坐标系下
    for(size_t leg(0); leg<4; ++leg){
        tmp_p[leg] = body_data->position + body_data->r_body.transpose() * (_hip_location[leg] + l_state[leg].p);
        tmp_v[leg] = body_data->r_body.transpose() * l_state[leg].v;
    }

    for(size_t i(0); i<3; ++i){ //三维
        fbm_state.bodyVelocity[i] = body_data->omega_body[i]; //转动
		fbm_state.bodyVelocity[i+3] = body_data->v_body[i];   //平动 //六维广义速度，在机身坐标系下的表示--------
        
        for(size_t leg(0); leg<4; ++leg){ //右前腿、左前腿、右后腿、左后腿
            fbm_state.q[3*leg + i] = l_state[leg].q[i];    //q-实际关节位置，12维
		    fbm_state.qd[3*leg + i] = l_state[leg].qd[i];  //qd-实际关节速度，12维
            
            fbm_state.p[3*leg + i] = tmp_p[leg][i];        //世界坐标系下，四个足端位置
            fbm_state.v[3*leg + i] = tmp_v[leg][i];        //世界坐标系下，四个足端速度

            _full_config[3*leg + i + 6] = fbm_state.q[3*leg + i];
        }
    }
    
    for(int i(0); i < 4; ++i){
        Jc[i] = dynamic->q_b_J[i];
        JcdotQdot[i] = dynamic->WBC_o_a[i];
    }//建立变量存储fb_dynamic计算的接触雅和JcdotQdot    

    /*
        动力学公式参数矩阵在WBIC.h的松弛优化中使用，对应原有框架，将_grav和_coriolis合并为孙师兄论文中C项，在fb_dynamics中使用牛顿-欧拉迭代求解；
        _A项对应孙师兄论文中H项，在fb_dynamics中计算；
    */
	_A = dynamic->H; //Mat18变量，惯性矩阵
	_C = dynamic->C; //Vec18变量，科里奥利力+重力
	_Ainv = _A.inverse();
}

/**
 * @brief 完成WBC中的所有的计算，
 * 调用_kin_wbc->FindConfiguration完成对机器人关节空间中的位置和速度项的计算，
 * 调用_wbic->UpdateSetting更新机体动力学参数，
 * 调用_wbic->MakeTorque完成WBC中的加速度计算和松弛优化，最终得到前馈扭矩
 */
template <typename T>
void WBC_Ctrl<T>::_ComputeWBC() {
  // KinWBC
  _kin_wbc->FindConfiguration(_full_config, _task_list, _contact_list, _des_jpos, _des_jvel);

  // WBIC
  _wbic->UpdateSetting(_A, _Ainv, _C);
  
  _wbic->MakeTorque(_tau_ff, _wbic_data);

}


/**
 * @brief 将计算的结果（_tau_ff、_des_jpos、_des_jvel）更新到腿部指令     
 * TODO：在locomotion_controller中调用获得WBC计算结果————————————————————————————————————————————
 * @param data 
 */
template<typename T>
void WBC_Ctrl<T>::_UpdateLegCMD(){

    for (size_t leg(0); leg < (size_t)4; ++leg) {
        for (size_t jidx(0); jidx < (size_t)3; ++jidx) {
            _WBC_legcmd[leg].tau_ff[jidx] = _tau_ff[(size_t)3 * leg + jidx];
			_WBC_legcmd[leg].q_des[jidx] = _des_jpos[(size_t)3 * leg + jidx];
			_WBC_legcmd[leg].qd_des[jidx] = _des_jvel[(size_t)3 * leg + jidx];

			// _WBC_legcmd[leg].kpJoint(jidx, jidx) = _Kp_joint[jidx]; //
			// _WBC_legcmd[leg].kdJoint(jidx, jidx) = _Kd_joint[jidx]; //在locomotion_controller中调用的时候会赋值。
        }
    }
}

/* 实现名为_WBC_legcmd
template<typename T>
struct WBC_legcmd
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Vec3<float> q_des, qd_des, tau_ff;  //期望位置、期望速度、前馈力矩
    Mat3<float> kp, kd;                 //比例控制系数、微分控制系数
};
*/

template class WBC_Ctrl<float>;
