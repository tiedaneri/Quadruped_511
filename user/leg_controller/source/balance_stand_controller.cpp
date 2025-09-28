/*
 * @Author: sun
 * @Date: 2021-04-11 04:45:44
 * @LastEditTime: 2021-04-13 20:27:23
 * @LastEditors: Please set LastEditors
 * @Description: 机器人站立模式的平衡控制器源文件
 * @FilePath: /robot_software/user/leg_controller/source/balance_stand_controller.cpp
 */

#include "balance_stand_controller.h"
#include "robot.h"
#include "Math/orientation_tools.h"
#include <stdio.h>//包含"std::"和"printf"定义

/**
 * @description: 构造函数，初始化参数
 * @param {无}
 * @return {无}
 */
balance_stand_controller::balance_stand_controller(at9s_cmd* _rc_cmd, leg_driver* _leg_drv, body_state_estimator_data* _body_data)
{
    rc_cmd    = _rc_cmd;   //连接遥控器指令
    leg_drv   = _leg_drv;  //连接腿驱动
    body_data = _body_data;//连接机身状态数据
    
    init();//各参数初始化
}

/**
 * @description: 初始化参数，每次重新进入该模式时，要运行该函数有且只有一次！！
 * @param {无}
 * @return {无}
 */
void balance_stand_controller::init()
{
    //更新初始足端位置，为默认的站立位置
    for(int leg = 0; leg < 4; leg++)
        pb0_bf[leg] = robot::hip_location[leg] + robot::stand_fpos[leg];

    //初始期望位姿，该值与机器人当前状态对应，该值下该控制器无动作，
    //可防止初次进入该控制器时期望位姿与当前位姿差别过大，导致瞬间的大动作
    roll_des   = 0;
    pitch_des  = 0;
    yaw_des    = 0;
    height_des = 0;

    //姿态偏差初始为0
    roll_err_old  = 0;
    roll_err_I    = 0;
    pitch_err_old = 0;
    pitch_err_I   = 0;
}

/**
 * @description: 平衡站立控制
 * @param {无}
 * @return {无}
 */
void balance_stand_controller::run()
{
    float roll_rc   = 0;//遥控器roll指令
    float pitch_rc  = 0;//遥控器pitch指令
    float yaw_rc    = 0;//遥控器yaw指令
    float height_rc = 0;//遥控器机身高度指令

    float roll_PID  = 0;//roll PID控制量
    float pitch_PID = 0;//pitch PID控制量

    if(rc_cmd->SWD == DOW)
    {
        //开启遥控器控制期望位置，关闭姿态反馈，
        //注意不要同时调整两个及以上的变量，可能会出现角度无解的情况！！！！
        
        //从遥控器获取位姿指令
        roll_rc   = -rc_cmd->LRO * 0.8;
        pitch_rc  =  rc_cmd->RCO * 0.8;
        yaw_rc    =  rc_cmd->RRO * 0.8;
        height_rc =  rc_cmd->LCO * 0.18;

        //姿态偏差归零
        roll_err_old  = 0;
        roll_err_I    = 0;
        pitch_err_old = 0;
        pitch_err_I   = 0;
    }
    else
    {
        //关闭遥控器控制，启动姿态反馈
        //姿态反馈只对roll和pitch进行反馈控制
        
        //当前姿态偏差值
        float roll_err  = roll_rc  - body_data->rpy(0);
        float pitch_err = pitch_rc - body_data->rpy(1);
        //更新比例控制量
        float roll_err_P  = roll_err  * 0.016;
        float pitch_err_P = pitch_err * 0.016;
        //更新积分控制量
        roll_err_I  += roll_err  * 0.01;
        pitch_err_I += pitch_err * 0.01;
        //更新微分控制量
        float roll_err_D  = (roll_err  - roll_err_old)  * 0.01;
        float pitch_err_D = (pitch_err - pitch_err_old) * 0.01;
        //更新PID控制量
        roll_PID  = roll_err_P  + roll_err_I  + roll_err_D;
        pitch_PID = pitch_err_P + pitch_err_I + pitch_err_D;
        //更新旧的误差值
        roll_err_old  = roll_err;
        pitch_err_old = pitch_err;
    }

    float roll_sum  = roll_rc  + roll_PID; //姿态指令控制加反馈控制
    float pitch_sum = pitch_rc + pitch_PID;//姿态指令控制加反馈控制
    //设置上限，防止角度无解
    if(roll_sum   >  0.4) roll_sum   =  0.4;
    if(roll_sum   < -0.4) roll_sum   = -0.4;
    if(pitch_sum  >  0.4) pitch_sum  =  0.4;
    if(pitch_sum  < -0.4) pitch_sum  = -0.4;
    
    float filter = 0.02;//期望位姿的低通滤波系数
    
    //对期望位姿进行低通滤波，防止变化过大产生震荡
    roll_des   = roll_sum  * filter + roll_des   * (1 - filter);//姿态指令控制加反馈控制
    pitch_des  = pitch_sum * filter + pitch_des  * (1 - filter);//姿态指令控制加反馈控制
    yaw_des    = yaw_rc    * filter + yaw_des    * (1 - filter);//姿态指令控制，无反馈
    height_des = height_rc * filter + height_des * (1 - filter);//机身高度指令控制，无反馈

    Vec3<float> rpy_des  = Vec3<float>(roll_des, pitch_des, yaw_des);//期望姿态欧拉角
    Quat<float> quat_des = ori::rpyToQuat(rpy_des);//期望姿态四元数
    //期望姿态旋转矩阵，方向为期望姿态到初始姿态的旋转，即反向旋转期望的姿态角度值，
    //由于旋转的对象为足端位置，所以机身的旋转角度刚好与之相反，为正的期望姿态角度值
    RotMat<float> r_des = ori::quaternionToRotationMatrix(quat_des);
    
    for(int leg = 0; leg < 4; leg++)
    {
        //期望足端相对机身的位置在机身坐标系下的表示
        Vec3<float> pbd_bf = r_des * (pb0_bf[leg] + Vec3<float>(0, 0, -height_des));
        //期望足端相对其腿基坐标系(hip)的位置在机身坐标系下的表示
        Vec3<float> pbd_hf = pbd_bf - robot::hip_location[leg];

        //将足端位置转换成关节位置指令
        robot::IK(pbd_hf, &(leg_drv->cmd[leg].q_des), leg);

        // if(leg == 1) printf("fpos tx: %f,%f,%f\n", pbd_hf(0), pbd_hf(1), pbd_hf(2));
        // if(leg == 0) printf("jpos tx: %f,%f,%f\n", leg_drv->cmd[leg].q_des(0), leg_drv->cmd[leg].q_des(1), leg_drv->cmd[leg].q_des(2));
    }
}
