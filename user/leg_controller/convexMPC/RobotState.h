/*
 * @Author: your name
 * @Date: 2021-10-04 04:07:08
 * @LastEditTime: 2021-10-04 04:29:17
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /robot_software/user/leg_controller/convexMPC/RobotState.h
 */
#ifndef _RobotState
#define _RobotState

#include <eigen3/Eigen/Dense>
#include "common_types.h"

using Eigen::Matrix;
using Eigen::Quaternionf;

#include "common_types.h"
class RobotState
{
    public:
        void set(flt* p, flt* v, flt* q, flt* w, flt* Fex, flt* r, flt yaw);
        void set(flt* p, flt* v, flt* q, flt* w, flt* r, flt yaw);
        //void compute_rotations();
        void print();
        Matrix<fpt,3,1> p,v,w,Fex;
        Matrix<fpt,3,4> r_feet;
        Matrix<fpt,3,1> r_ex; 
        Matrix<fpt,3,3> R;
        Matrix<fpt,3,3> R_yaw;
        Matrix<fpt,3,3> I_body;
        Quaternionf q;
        fpt yaw;
        fpt m = 10.5;
        // fpt m = 17.426;
        //fpt m = 9;      //mini
        //fpt m = 50.236; //DH
    //private:
};
#endif
