/*
 * @Author: your name
 * @Date: 2021-10-04 04:07:08
 * @LastEditTime: 2021-10-04 04:28:57
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /robot_software/user/leg_controller/convexMPC/RobotState.cpp
 */
#include "RobotState.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <math.h>

using std::cout;
using std::endl;

/**
 * @brief  重载，给robot2用的set函数
 */
void RobotState::set(flt* p_, flt* v_, flt* q_, flt* w_,  flt* Fex_, flt* r_,flt yaw_)
{
    for(u8 i = 0; i < 3; i++)
    {
        this->p(i) = p_[i];
        this->v(i) = v_[i];
        this->w(i) = w_[i];
        this->Fex(i) = Fex_[i];
    }
    this->q.w() = q_[0];
    this->q.x() = q_[1];
    this->q.y() = q_[2];
    this->q.z() = q_[3];
    this->yaw = yaw_;

    for(u8 rs = 0; rs < 3; rs++)
        for(u8 c = 0; c < 4; c++)
            this->r_feet(rs,c) = r_[rs*4 + c];

    R = this->q.toRotationMatrix();
    fpt yc = cos(yaw_);
    fpt ys = sin(yaw_);

    R_yaw <<  yc,  -ys,  0,
              ys,  yc,   0,
               0,   0,   1;

    r_ex << 0., 0., 0.158; //世界系下机身中心到外力作用点向量（与机身系下的一样）

    // I_body << 0.317, 0.000, -0.053,
	//           0.000, 1.117,  0.000,
	//          -0.053, 0.000,  1.139;
    I_body << 0.070, 0.000, 0.000,
	          0.000, 0.260, 0.000,
	          0.000, 0.000, 0.242;
}

/**
 * @brief  重载，给robot1用的set函数，比robot2少了Fex传参和成员变量r_ex
 */
void RobotState::set(flt* p_, flt* v_, flt* q_, flt* w_, flt* r_,flt yaw_)
{
    for(u8 i = 0; i < 3; i++)
    {
        this->p(i) = p_[i];
        this->v(i) = v_[i];
        this->w(i) = w_[i];
    }
    this->q.w() = q_[0];
    this->q.x() = q_[1];
    this->q.y() = q_[2];
    this->q.z() = q_[3];
    this->yaw = yaw_;

    for(u8 rs = 0; rs < 3; rs++)
        for(u8 c = 0; c < 4; c++)
            this->r_feet(rs,c) = r_[rs*4 + c];

    R = this->q.toRotationMatrix();
    fpt yc = cos(yaw_);
    fpt ys = sin(yaw_);

    R_yaw <<  yc,  -ys,   0,
              ys,  yc,   0,
               0,   0,   1;

    // I_body << 0.317, 0.000, -0.053,
	//           0.000, 1.117,  0.000,
	//          -0.053, 0.000,  1.139;
    I_body << 0.070, 0.000, 0.000,
	          0.000, 0.260, 0.000,
	          0.000, 0.000, 0.242;
}

void RobotState::print()
{
   cout<<"Robot State:"<<endl<<"Position\n"<<p.transpose()
       <<"\nVelocity\n"<<v.transpose()<<"\nAngular Veloctiy\n"
       <<w.transpose()<<"\nRotation\n"<<R<<"\nYaw Rotation\n"
       <<R_yaw<<"\nFoot Locations\n"<<r_feet<<"\nInertia\n"<<I_body<<endl;
}



