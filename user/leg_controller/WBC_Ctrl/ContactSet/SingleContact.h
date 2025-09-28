/*
 * @Author: Si
 * @Date: 2024-06-26 00:21:25
 * @LastEditTime: 2024-10-09 11:10:36
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /robot_software_1/user/leg_controller/WBC_Ctrl/ContactSet/SingleContact.h
 */
#ifndef SINGLE_CONTACT
#define SINGLE_CONTACT

#include "ContactSpec.h"

template <typename T>
class SingleContact : public ContactSpec<T>{
public:
    SingleContact();
    ~SingleContact() {}

    void setMaxFz(T max_Fz) {_max_Fz = max_Fz;}

protected:
    T _max_Fz;
    int _dim_U; //Uf_矩阵的行数，实际意义？？？？？？？？？？？？？？？

    virtual bool _UpdateJc(Mat3_18<float> Jc);
	virtual bool _UpdateJcDotQdot(Vec3<float> JcdotQdot);
	virtual bool _UpdateUf(){return true;} //每个脚都一样，无需更新
	virtual bool _UpdateInequalityVector();


};

#endif /*SINGLE_CONTACT*/
