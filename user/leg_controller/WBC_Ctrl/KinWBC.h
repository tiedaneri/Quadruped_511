/*
 * @Author: Si
 * @Date: 2024-06-11 09:39:04
 * @LastEditTime: 2024-09-19 17:05:28
 * @LastEditors: Please set LastEditors
 * @Description: 速度和位置WBC
 * @FilePath: /robot_software_1/user/leg_controller/WBC_Ctrl/KinWBC.h
 */

#ifndef KINEMATICS_WHOLE_BODY_CONTROL
#define KINEMATICS_WHOLE_BODY_CONTROL

#include "ContactSpec.h"
#include "Task.h"
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/SVD>
#include "Utilities/pseudoInverse.h"

#include <vector>
#include <cppTypes.h>

template <typename T>
class KinWBC{
public:    
    /**
     * @description: 构造函数
     * @param {num_qdot} 总自由度18
     * @return {*}
     */
    KinWBC(size_t num_qdot)
    : num_qdot_(num_qdot), num_act_joint_(num_qdot - 6)
    {
        I_mtx = DMat<T>::Identity(num_qdot_, num_qdot_);
    }

    ~KinWBC(){}
    
    /**
     * @description: WBC优先级计算位置、速度（delta_q, qdot）,赋值给jpos_cmd和jvel_cmd;
     * @param {*}
     * @return {*}
     */
    bool FindConfiguration(const DVec<T>& curr_config,
                           const std::vector<Task<T>*>& task_list,
                           const std::vector<ContactSpec<T>*>& contact_list,
                           DVec<T>& jpos_cmd,
                           DVec<T>& jvel_cmd)
    {
        //构建接触雅Jc（任务1任务雅）
        DMat<T> Nc(num_qdot_, num_qdot_);
        Nc.setIdentity();// 接触雅各比Jc的零空间阵Nc设为单位阵

        if(contact_list.size() > 0){//有接触腿
            DMat<T> Jc, Jc_i;
            contact_list[0]->getContactJacobian(Jc);//先处理第一条腿
            size_t Jc_rows = Jc.rows();

            for(size_t i(1); i < contact_list.size(); ++i){
                contact_list[i]->getContactJacobian(Jc_i);
                size_t Jc_new_rows = Jc_i.rows();
                Jc.conservativeResize(Jc_rows + Jc_new_rows, num_qdot_);//保留原有数值，改变动态矩阵Jc大小
                Jc.block(Jc_rows, 0, Jc_new_rows, num_qdot_) = Jc_i;//把新的Jc_i堆到Jc矩阵下边
                Jc_rows += Jc_new_rows;
            }//至此求得所有接触腿的堆叠雅，即接触雅Jc（任务1任务雅）

            _BuildProjectionMatrix(Jc, Nc); //计算零空间阵Nc，（20-1），计算任务2时用到
        }

        //任务2-机身转动任务 
        DVec<T> delta_q, qdot;
        DMat<T> Jt, JtPre, JtPre_pinv, N_nx, N_pre;

        Task<T>* task = task_list[0];//任务2
        task->getTaskJacobian(Jt);// 任务2的任务雅
        JtPre = Jt * Nc;//(19)-1, i = 1
        _PseudoInverse(JtPre, JtPre_pinv);

        delta_q = JtPre_pinv * (task->getPosError());// 对于任务2，prev_delta_q和prev_qdot均为0，简化(16)(17)
        qdot = JtPre_pinv * (task->getDesVel()); // 计算出任务2的delta_q和qdot

        DVec<T> prev_delta_q = delta_q;
	    DVec<T> prev_qdot = qdot;//迭代更新

        _BuildProjectionMatrix(JtPre, N_nx); // (20)-2
        N_pre = Nc * N_nx; // (19)-2:计算N_2(任务3要用)堆叠任务雅的零空间阵

        for (size_t i(1); i < task_list.size(); ++i) { // 从任务3开始遍历任务(3和4)
            task = task_list[i];
            task->getTaskJacobian(Jt);
            JtPre = Jt * N_pre;
            _PseudoInverse(JtPre, JtPre_pinv);

            delta_q = prev_delta_q + JtPre_pinv * (task->getPosError() - Jt * prev_delta_q); // (16)
            qdot = prev_qdot + JtPre_pinv * (task->getDesVel() - Jt * prev_qdot); // (17)
            
            prev_delta_q = delta_q;
            prev_qdot = qdot;//迭代更新
            
            _BuildProjectionMatrix(JtPre, N_nx);// (20)-2
            N_pre *= N_nx; // (19)-2:堆叠任务雅的零空间阵
	    } // 至此求得满足所有任务的delta_q, qdot

        for(size_t i(0); i < num_act_joint_; ++i){
            jpos_cmd[i] = curr_config[i + 6] + delta_q[i + 6];// (24)
            jvel_cmd[i] = qdot[i + 6];
        }
        return true;
    }

private:

    /**
     * @description: 计算矩阵右加号逆
     * @param {*}
     * @return {*}
     */
    void _PseudoInverse(const DMat<T> J, DMat<T>& Jinv, double threshold_ = 0.001){
        pseudoInverse(J, threshold_, Jinv);
    }
    
    /**
     * @description: 计算零空间投影阵Nc
     * @param {*}
     * @return {*}
     */
    void _BuildProjectionMatrix(const DMat<T>& J, DMat<T>& N){
        DMat<T> J_Pinv;
        _PseudoInverse(J, J_Pinv);
        N = I_mtx - J_Pinv * J;
    }

    
    size_t num_qdot_; //总自由度18
	size_t num_act_joint_; //被驱动的关节数目12
	DMat<T> I_mtx;
};


#endif /*KINEMATICS_WHOLE_BODY_CONTROL*/