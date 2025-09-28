/*
 * @Author: Si
 * @Date: 2024-06-11 09:39:04
 * @LastEditTime: 2024-10-09 10:58:28
 * @LastEditors: Please set LastEditors
 * @Description: 加速度WBC和松弛优化计算，不使用继承WBC父类    ToDo:MakeTorque函数
 * @FilePath: /robot_software_1/user/leg_controller/WBC_Ctrl/WBIC.h
 */

#ifndef WHOLE_BODY_IMPULSE_CONTROL
#define WHOLE_BODY_IMPULSE_CONTROL

#include "ContactSpec.h"
#include "Task.h"
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/SVD>
#include "Utilities/pseudoInverse.h"
#include "QuadProg++.hh"

#include <vector>
#include <cppTypes.h>

template <typename T>
class WBIC_ExtraData{
public:
    // Output
	DVec<T> _opt_result;
	DVec<T> _qddot;
	DVec<T> _Fr;

	// Input
	DVec<T> _W_floating; //松弛优化权重阵Q1，在WBC_Ctrl构造函数中初始化
	DVec<T> _W_rf;       //松弛优化权重阵Q2，在WBC_Ctrl构造函数中初始化

	WBIC_ExtraData() {}
	~WBIC_ExtraData() {}
};

template <typename T>
class WBIC{
public:
    /**
     * @description: 构造函数
     * @param {num_qdot} 总自由度18
     * @return {*}
     */
    WBIC(size_t num_qdot,
         const std::vector<ContactSpec<T>*>* contact_list,
         const std::vector<Task<T>*>* task_list)
    :num_qdot_(num_qdot), num_act_joint_(num_qdot - 6), _dim_floating(6)
    {
        Sa_ = DMat<T>::Zero(num_act_joint_, num_qdot_);
        Sv_ = DMat<T>::Zero(6, num_qdot_);
        Sa_.block(0, 6, num_act_joint_, num_act_joint_).setIdentity();
        Sv_.block(0, 0, 6, 6).setIdentity();

        _contact_list = contact_list;
        _task_list = task_list;
        _eye = DMat<T>::Identity(num_qdot_, num_qdot_);
        _eye_floating = DMat<T>::Identity(_dim_floating, _dim_floating);
    }

    ~WBIC(){}

    /**
     * @brief 更新拉格朗日动力学参数
     * @param {A, Ainv, C, extra_setting}
     * @return {无}
     */
    void UpdateSetting(const DMat<T>& A,
                       const DMat<T>& Ainv,
                       const DVec<T>& C,
                       void* extra_setting = NULL) 
    {
        A_ = A;
        Ainv_ = Ainv;
        cori_ = C; //科+重力
        b_updatesetting_ = true;

        (void)extra_setting;
    }

    /**
     * @brief 完成接触雅Jc、JcDotQdot、摩擦锥等的初始化  
     * @param {无}
     * @return {无}
     */
    void _ContactBuilding(){

        DMat<T> Jc;
        DVec<T> JcDotQdot; 
        DMat<T> Uf;
        DVec<T> Uf_ieq_vec; //过程量，单个腿的量，后边要被叠起来的

        size_t dim_accumul_rf, dim_accumul_uf;

        (*_contact_list)[0]->getContactJacobian(Jc); //从动力学获得Jc
        (*_contact_list)[0]->getJcDotQdot(JcDotQdot); //从动力学获得JcDotQdot
        (*_contact_list)[0]->getRFConstraintMtx(Uf);  //摩擦锥中构造阵，每个脚都一样
        (*_contact_list)[0]->getRFConstraintVec(Uf_ieq_vec);  //摩擦锥中最小阵，每个脚都一样

        dim_accumul_rf = (*_contact_list)[0]->getDim(); //力维度,3,fx、fy、fz
        dim_accumul_uf = (*_contact_list)[0]->getDimRFConstraint(); //不等式维度，在接触任务中初始化为6
        
        _Jc.block(0, 0, dim_accumul_rf, num_qdot_) = Jc;
        _JcDotQdot.head(dim_accumul_rf) = JcDotQdot;
        _Uf.block(0, 0, dim_accumul_uf, dim_accumul_rf) = Uf;
        _Uf_ieq_vec.head(dim_accumul_uf) = Uf_ieq_vec;

        _Fr_des.head(dim_accumul_rf) = (*_contact_list)[0]->getRFDesired(); //需要在接触基类中实现，获取MPC计算的足端力

        size_t dim_new_rf, dim_new_uf;
        for(size_t i(1); i < (*_contact_list).size(); ++i){
            (*_contact_list)[i]->getContactJacobian(Jc); 
            (*_contact_list)[i]->getJcDotQdot(JcDotQdot);
            (*_contact_list)[i]->getRFConstraintMtx(Uf);  
            (*_contact_list)[i]->getRFConstraintVec(Uf_ieq_vec); //跟上边第0只脚一样的操作

            dim_new_rf = (*_contact_list)[i]->getDim();
            dim_new_uf = (*_contact_list)[i]->getDimRFConstraint();

            _Jc.block(dim_accumul_rf, 0, dim_new_rf, num_qdot_) = Jc;
            _JcDotQdot.segment(dim_accumul_rf, dim_new_rf) = JcDotQdot;
            _Uf.block(dim_accumul_uf, dim_accumul_rf, dim_new_uf, dim_new_rf) = Uf;
            _Uf_ieq_vec.segment(dim_accumul_uf, dim_new_uf) = Uf_ieq_vec;

            _Fr_des.segment(dim_accumul_rf, dim_new_rf) = (*_contact_list)[i]->getRFDesired(); // 拿MPC计算的最优足端力
            
            dim_accumul_rf += dim_new_rf;
            dim_accumul_uf += dim_new_uf;
        }

    }
    
    /**
     * @description: WBIC优先级计算加速度（qddot）,并进行松弛优化。
     * @param {*}
     * @return {*}
     */
    void MakeTorque(DVec<T>& cmd, void* extra_input)
    {
        if(!b_updatesetting_) printf("[Wanning] WBIC setting is not done\n");
        if(extra_input) _data = static_cast<WBIC_ExtraData<T>*>(extra_input);

        // resize G, g0, CE, ce0, CI, ci0
        _SetOptimizationSize();
        _SetCost();

        // Contact
        DVec<T> qddot_pre;
        DMat<T> JcBar;
        DMat<T> Npre;
        
        if(_dim_rf > 0){ //如果有支撑腿。_dim_rf--支撑的维度，大小是(3 * 支撑腿的数目)
            _ContactBuilding();

            _SetInEqualityConstraint();// 不等式约束

            _WeightedInverse(_Jc, Ainv_, JcBar);
            qddot_pre = JcBar * (-_JcDotQdot);
            Npre = _eye - JcBar * _Jc;
        }
        else{
            qddot_pre = DVec<T>::Zero(num_qdot_);
		    Npre = _eye;
        }

        // Task
        Task<T>* task;
        DMat<T> Jt, JtBar, JtPre;
        DVec<T> JtDotQdot, xddot;

        for(size_t i(0); i < (*_task_list).size(); ++i){
            task = (*_task_list)[i];

            task->getTaskJacobian(Jt); //需要在任务基类中实现
            task->getTaskJacobianDotQdot(JtDotQdot); //需要在任务基类中实现
            task->getCommand(xddot);//加速度的规划（22）

            JtPre = Jt * Npre;
            _WeightedInverse(JtPre, Ainv_, JtBar);
            qddot_pre += JtBar * (xddot - JtDotQdot - Jt * qddot_pre);
            Npre *= (_eye - JtBar * JtPre);
        }// 至此求得满足所有任务的 qddot = qddot_pre

	    _SetEqualityConstraint(qddot_pre);// 等式约束

        // 松弛优化函数
        T f = solve_quadprog(G, g0, CE, ce0, CI, ci0, z);
        (void)f;

        for (size_t i(0); i < _dim_floating; ++i) qddot_pre[i] += z[i]; //保存优化结果中qddot
        _GetSolution(qddot_pre, cmd); //把松弛优化后的结果整合得到关节扭矩指令

        _data->_opt_result = DVec<T>(_dim_opt);
        for(size_t i(0); i < _dim_opt; ++i){
            _data->_opt_result[i] = z[i]; // 优化结果
        }
    }

private:

    /**
     * @brief 设置优化问题的规模，确定优化矩阵大小
     * 
     */
    void _SetOptimizationSize(){
        _dim_rf = 0; //支撑的维度
        _dim_Uf = 0; //摩擦锥维度
        for (size_t i(0); i < (*_contact_list).size(); ++i) {
            _dim_rf += (*_contact_list)[i]->getDim(); //支撑的维度，大小是(3 * 支撑腿的数目)
            _dim_Uf += (*_contact_list)[i]->getDimRFConstraint();
	    }

        _dim_opt = _dim_floating + _dim_rf;
	    _dim_eq_cstr = _dim_floating;

        // Matrix Setting
        G.resize(0., _dim_opt, _dim_opt);
        g0.resize(0., _dim_opt);
        CE.resize(0., _dim_opt, _dim_eq_cstr);
        ce0.resize(0., _dim_eq_cstr);

        // Eigen Matrix Setting
	    _dyn_CE = DMat<T>::Zero(_dim_eq_cstr, _dim_opt);
        _dyn_ce0 = DVec<T>(_dim_eq_cstr);

        if(_dim_rf > 0) {
            CI.resize(0., _dim_opt, _dim_Uf);
            ci0.resize(0., _dim_Uf);
            _dyn_CI = DMat<T>::Zero(_dim_Uf, _dim_opt);
            _dyn_ci0 = DVec<T>(_dim_Uf);

            _Jc = DMat<T>(_dim_rf, num_qdot_);
            _JcDotQdot = DVec<T>(_dim_rf);
            _Fr_des = DVec<T>(_dim_rf);

            _Uf = DMat<T>(_dim_Uf, _dim_rf);
            _Uf.setZero();
            _Uf_ieq_vec = DVec<T>(_dim_Uf);
	    } 
        else{
            CI.resize(0., _dim_opt, 1);
		    ci0.resize(0., 1);
        }
    }

    /**
     * @brief 松弛优化的cost函数，设置两个权重阵
     * 
     */
    void _SetCost(){
        size_t idx_offset(0);
        // qddot的权重阵，Q1
        for (size_t i(0); i < _dim_floating; ++i) {
            G[i + idx_offset][i + idx_offset] = _data->_W_floating[i];
        }

        idx_offset += _dim_floating;
        // fc的权重阵，Q2
        for (size_t i(0); i < _dim_rf; ++i) {
            G[i + idx_offset][i + idx_offset] = _data->_W_rf[i];
        }
    }

    /**
     * @brief 松弛优化-设置不等式约束：摩擦锥-------------CI和ci0
     */
    void _SetInEqualityConstraint(){
        _dyn_CI.block(0, _dim_floating, _dim_Uf, _dim_rf) = _Uf;
        _dyn_ci0 = _Uf_ieq_vec - _Uf * _Fr_des;

        for(size_t i(0); i < _dim_Uf; ++i){
            for(size_t j(0); j < _dim_opt; ++j){
                CI[j][i] = _dyn_CI(i, j);
            }
            ci0[i] = -_dyn_ci0[i];
        }
    }


    /**
     * @brief 松弛优化-设置等式约束：浮基机身动力学-------------CE和ce0
     * @param {qddot}
     */
    void _SetEqualityConstraint(const DVec<T>& qddot){
        if(_dim_rf > 0){
            _dyn_CE.block(0, 0, _dim_eq_cstr, _dim_floating) = A_.block(0, 0, _dim_floating, _dim_floating);
            _dyn_CE.block(0, _dim_floating, _dim_eq_cstr, _dim_rf) = -Sv_ * _Jc.transpose();
            _dyn_ce0 = -Sv_ * (A_ * qddot + cori_ - _Jc.transpose() * _Fr_des);
        } 
        else{
            _dyn_CE.block(0, 0, _dim_eq_cstr, _dim_floating) = A_.block(0, 0, _dim_floating, _dim_floating);
            _dyn_ce0 = -Sv_ * (A_ * qddot + cori_);
        }

        for(size_t i(0); i < _dim_eq_cstr; ++i){
            for (size_t j(0); j < _dim_opt; ++j){
                CE[j][i] = _dyn_CE(i, j);
            }
            ce0[i] = -_dyn_ce0[i];
        }
    }
    
    /**
     * @description: 把松弛优化后的结果整合得到关节扭矩指令
     * @param {qddot}
     * @param {cmd}
     */
    void _GetSolution(const DVec<T>& qddot, DVec<T>& cmd){
        DVec<T> tot_tau;
        if(_dim_rf > 0){ //_dim_rf--支撑的维度，大小是(3 * 支撑腿的数目)
            _data->_Fr = DVec<T>(_dim_rf); 
            for (size_t i(0); i < _dim_rf; ++i){
                _data->_Fr[i] = z[i + _dim_floating] + _Fr_des[i]; // 松弛优化后的力
            } 
            tot_tau = A_ * qddot + cori_ - _Jc.transpose() * _data->_Fr;
        }
        else{
            tot_tau = A_ * qddot + cori_;
        }
        _data->_qddot = qddot;
        cmd = tot_tau.tail(num_act_joint_); // 倒数num_act_joint_（12）个成员，12关节
    }


    /**
     * @description: compute dynamically consistent pseudo-inverse
     * @param {Winv} 权重的逆，对应论文里公式(23)
     * @return {*}
     */
    void _WeightedInverse(const DMat<T> J, const DMat<T>& Winv, DMat<T>& Jinv, double threshold_ = 0.0001){
        DMat<T> lambda(J * Winv * J.transpose());
        DMat<T> lambda_inv;
        pseudoInverse(lambda, threshold_, lambda_inv);
        Jinv = Winv * J.transpose() * lambda_inv;
    }

    size_t num_qdot_; //总自由度18
	size_t num_act_joint_; //被驱动的关节数目12

    DMat<T> Sa_;  // Actuated joint
	DMat<T> Sv_;  // Virtual joint
    
    const std::vector<ContactSpec<T>*>* _contact_list;//接触腿列表
	const std::vector<Task<T>*>* _task_list;//任务列表

    size_t _dim_floating; //浮动基座的自由度

	DMat<T> _eye;
	DMat<T> _eye_floating;

    DMat<T> A_;
	DMat<T> Ainv_;
	DVec<T> cori_; //科+重力

    bool b_updatesetting_; //更新拉格朗日动力学参数函数的 相关bool变量
    bool b_internal_constraint_;
    
    DMat<T> _Uf; //不等式约束构造阵，在接触任务.cpp中定义，是一个常量矩阵
	DVec<T> _Uf_ieq_vec; //不等式约束最小阵，在接触任务.cpp中定义，是一个常量矩阵

    DMat<T> _Jc;
	DVec<T> _JcDotQdot;
    
	DVec<T> _Fr_des; // MPC计算的最优足端力
	
    WBIC_ExtraData<T>* _data;

    size_t _dim_opt;
    size_t _dim_eq_cstr;

    size_t _dim_rf; // 表示支撑的维度,在_SetOptimizationSize()中赋值
	size_t _dim_Uf; // 表示总摩擦锥的维度

    GolDIdnani::GVect<double> z; // 优化结果
	// Cost
	GolDIdnani::GMatr<double> G;
	GolDIdnani::GVect<double> g0;

	// Equality
	GolDIdnani::GMatr<double> CE;
	GolDIdnani::GVect<double> ce0;

	// Inequality
	GolDIdnani::GMatr<double> CI;
	GolDIdnani::GVect<double> ci0;

    DMat<T> _dyn_CE;
	DVec<T> _dyn_ce0;
	DMat<T> _dyn_CI;
	DVec<T> _dyn_ci0;

};


#endif /*WHOLE_BODY_IMPULSE_CONTROL*/
