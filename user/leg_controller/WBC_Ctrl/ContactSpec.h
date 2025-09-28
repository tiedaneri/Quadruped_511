/*
 * @Author: Si
 * @Date: 2024-06-25 19:16:13
 * @LastEditTime: 2024-09-18 11:08:18
 * @LastEditors: Please set LastEditors
 * @Description: 定义了接触力任务基类用于接触力任务的描述与构建，其中定义了任务矩阵维度，Jacobian更新方法，期望更新方法，
                 在WBC_Ctrl/ContactSet文件夹下通过继承该基类定义了单刚体简化动力学模型接触任务。
 * @FilePath: /robot_software_1/user/leg_controller/WBC_Ctrl/ContactSpec.h
 */


#ifndef CONTACT_SPEC
#define CONTACT_SPEC

#include <cppTypes.h>

#define Contact ContactSpec<T>

template <typename T> 
class ContactSpec{
public:
    ContactSpec(size_t dim)
    :dim_contact_(dim), b_set_contact_(false){
        idx_Fz_ = dim - 1;
        Fr_des_ = DVec<T>::Zero(dim);
    }

    virtual ~ContactSpec(){}

    size_t getDim() const { return dim_contact_; }//3
    size_t getDimRFConstraint() const { return Uf_.rows(); }
    size_t getFzIndex() const { return idx_Fz_; }

    void getContactJacobian(DMat<T>& Jc) { Jc = Jc_; }
    void getJcDotQdot(DVec<T>& JcDotQdot) { JcDotQdot = JcDotQdot_; }
    void UnsetContact() { b_set_contact_ = false; }    

    void getRFConstraintMtx(DMat<T>& Uf) { Uf = Uf_; }
    void getRFConstraintVec(DVec<T>& ieq_vec) { ieq_vec = ieq_vec_; }
    const DVec<T>& getRFDesired() { return Fr_des_; }
    
    void setRFDesired(const DVec<T>& Fr_des) { Fr_des_ = Fr_des; } //顶层调用获得地面反作用力

    bool UpdateContactSpec(Mat3_18<float> Jc, Vec3<float> JcdotQdot) { //顶层LocomotionCtrl调用更新任务
      _UpdateJc(Jc);
      _UpdateJcDotQdot(JcdotQdot);
      _UpdateUf();
      _UpdateInequalityVector();
      b_set_contact_ = true;
      return true;
    }

protected:
    virtual bool _UpdateJc(Mat3_18<float> Jc) = 0;
    virtual bool _UpdateJcDotQdot(Vec3<float> JcdotQdot) = 0;
    virtual bool _UpdateUf() = 0;
    virtual bool _UpdateInequalityVector() = 0;
    
    int idx_Fz_; //摩擦锥中足端Fz的序号
    DMat<T> Uf_; ////摩擦锥中构造阵
    DVec<T> ieq_vec_; //摩擦锥中不等式最小值阵
    
    DVec<T> Fr_des_; //最优足端力，在顶层调用中赋值

    DMat<T> Jc_; //接触矩阵，3x18
    DVec<T> JcDotQdot_; //接触JcDotQdot,3x1
    size_t dim_contact_; //接触维度，3
    bool b_set_contact_;
};


#endif /*CONTACT_SPEC*/