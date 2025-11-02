/*
 * @Author: sun
 * @Date: 2021-04-13 23:14:42
 * @LastEditTime: 2025-09-24 11:09:45
 * @LastEditors: Please set LastEditors
 * @Description: 机器人运动模式控制器源文件
 * @FilePath: /robot_software/user/leg_controller/source/locomotion_controller.cpp
 */

#include "locomotion_controller.h"
#include "robot.h"
#include "Math/orientation_tools.h"
#include "convexMPC_interface.h"
#include <stdio.h>//包含"std::"和"printf"定义
#include <Utilities/Timer.h>//记录程序运行时间

#define mpc_step 10 //执行一次mpc预测步长

/**
 * @description: 构造函数，初始化参数
 * @param {无}
 * @return {无}
 */
locomotion_controller::locomotion_controller(at9s_cmd* _rc_cmd, leg_driver* _leg_drv, body_state_estimator_data* _body_data, fb_dynamic* _dynamic, usb_device_data* _usb_data)
{
    rc_cmd    = _rc_cmd;   //连接遥控器指令
    leg_drv   = _leg_drv;  //连接腿驱动
    body_data = _body_data;//连接机身状态数据
    usb_data = _usb_data;  //连接USB设备数据
    dynamic = _dynamic;    //连接浮基动力学

#ifdef WBCANDMPC
    //WBC模块
    _wbc_ctrl = new LocomotionCtrl<float>(dynamic, body_data, leg_drv->state);
    _wbc_data = new LocomotionCtrlData<float>();
#endif /* WBCANDMPC */

    init();//各参数初始化
}

/**
 * @description: 初始化参数，每次重新进入该模式时，要运行该函数有且只有一次！！
 * @param {无}
 * @return {无}
 */
void locomotion_controller::init()
{
    trot_gait.init();//初始化参数

    trot_gait.c_gait = 9;//一个dt_gait，控制器执行的次数
    trot_gait.T = 0.4;//一个完整步态周期时间

#ifdef ROBOT2
    trot_gait.T = 0.25;//一个完整步态周期时间    
#endif

    //各腿相位偏置
    trot_gait.phase_offset[0] = 0;
    trot_gait.phase_offset[1] = 0.5;
    trot_gait.phase_offset[2] = 0.5;
    trot_gait.phase_offset[3] = 0;
    //各腿支撑相占比
    trot_gait.phase_stance[0] = 0.5;
    trot_gait.phase_stance[1] = 0.5;
    trot_gait.phase_stance[2] = 0.5;
    trot_gait.phase_stance[3] = 0.5;
    trot_gait.set();//生效步态设置,当前参数为对角步态
    
    //站立足端位置的偏置，用于调整重心等
    Vec3<float> stand_offset[4] = 
    {
        Vec3<float>(0.005, -0.03, 0),
        Vec3<float>(0.005,  0.03, 0),
        Vec3<float>(0.005, -0.03, 0),
        Vec3<float>(0.005,  0.03, 0)
    };
    
    for(int leg = 0; leg < 4; leg++)
    {
        //更新初始足端位置（在机身坐标系），为默认的站立位置加偏置
        pb0_bf[leg] = robot::hip_location[leg] + robot::stand_fpos[leg] + stand_offset[leg];
        first_stance[leg] = true;
        first_swing[leg]  = true;

        tra[leg].pos_initial = robot::stand_fpos[leg];
        tra[leg].p = robot::stand_fpos[leg];
        tra[leg].height = 0.03;

        contact_flag[leg] = true;//初始化接触状态

    }
        
    //初始期望速度，该值下该控制器无动作，
    //可防止初次进入该控制器时　期望速度与当前位姿差别过大，导致瞬间的大动作
    vx_des   = 0;
    vy_des   = 0;
    vyaw_des = 0;
    hf_des   = 0;

    //一个mpc步长的时间与步态规划执行的最小时间单位相同, 0.023s
    dtmpc = trot_gait.dt_gait;
    //x方向速度对z轴位置的影响初始为0
    x_comp_integral = 0;

    body_height = 0.29176;//机身目标高度

    for(int i = 0; i < 4; i++) f_ff[i] = Vec3<float>::Zero();

    mpc_real_dt = 0;

    setup_problem(dtmpc,mpc_step,0.4,80);//配置MPC

}

#ifdef WBCANDMPC
/**
 * @description: 运动控制
 * @param {无}
 * @return {无}
 */
void locomotion_controller::run()
{
    trot_gait.run();//执行步态规划，实际10个控制周期执行一次
    
    //目前已经在leg_controller中更新为足端接触状态
    for(int leg = 0; leg < 4; leg++)
    {
        //更新状态估计中的足端接触状态数据, 进入支撑相的时间占总支撑相时间的比值, (float)0-1
        body_data->contact_state(leg) = trot_gait.stance_state[leg];
    }
    
    //遥控器期望前后方向速度指令
    //遥控器期望左右方向速度指令
    //遥控器期望偏航角速度指令
    //遥控器期望抬腿高度指令
    float vx_rc   = rc_cmd->RCO * vx_max;
    float vy_rc   = rc_cmd->LRO * vy_max;
    float vyaw_rc = rc_cmd->RRO * vyaw_max;
    float hf_rc   = rc_cmd->VB  * hight_max;
    
    //UP:无速度指令时静止
    //DOW:无速度指令时保持原地踏步
    // if(rc_cmd->SWD == UP)
    // {
    //     float cmd_limit = 0.002;//动作的最小值
    //     //无速度指令时静止
    //     if(fabs(rc_cmd->RCO) < cmd_limit && fabs(rc_cmd->LRO) < cmd_limit && fabs(rc_cmd->RRO) < cmd_limit)
    //     {
    //         vx_rc   = 0;
    //         vy_rc   = 0;
    //         vyaw_rc = 0;
    //         hf_rc   = 0;
    //     }
    // }
    

    //低通滤波
    float filter = 0.03;//期望机器人状态指令滤波系数0.03
#ifdef ROBOT2
    filter = 0.005;     //二号机器人指令滤波系数0.005
#endif
    vx_des   = vx_rc   * filter + vx_des   * (1 - filter);
    vy_des   = vy_rc   * filter + vy_des   * (1 - filter);
    vyaw_des = vyaw_rc * filter + vyaw_des * (1 - filter);
    hf_des   = hf_rc   * filter   + hf_des * (1 - filter);

    for(int leg = 0; leg < 4; leg++)//基于实际触地状态的复杂地形运动控制策略
    {
        if(trot_gait.stance_flag[leg])//规划的支撑相
        {
            stance_control_flag[leg] = true;//支撑控制
        }
        else//规划的摆动相
        {
            if(trot_gait.t_swing[leg] < 0.5)//前半段-----有问题,感觉应该是swing_state[leg]，现在的写法使力判断的接触状态无效
            {
                stance_control_flag[leg] = false;//摆动控制
            }
            else//后半段
            {
                if(contact_flag[leg])//实际已经触地
                {
                    stance_control_flag[leg] = true;//支撑控制
                }
                else//实际未触地
                {
                    stance_control_flag[leg] = false;//摆动控制
                }
            }
        }
    }
    // for(int leg = 0; leg < 4; leg++)//基于规划触地状态的运动控制策略
    // {
    //     stance_control_flag[leg] = trot_gait.stance_flag[leg];
    // }

    //MPC计算足端力,每过一个dtmpc或dt_gait，执行一次run_MPC()
    if(trot_gait.c == 1) run_MPC(); //在第一个c时执行run_MPC(), 在第10个c执行gait_plan::run()

    //***********************************************WBC质心规划***********************************************
    Vec3<float> v_des_world = body_data->r_body.transpose() * Vec3<float>(vx_des, vy_des, 0);//世界系下，由遥控器输入左乘旋转矩阵
    Vec3<float> world_position_desired = body_data->position + robot::dt * v_des_world;
    float _yaw_des = body_data->rpy[2] + robot::dt * vyaw_rc;

    // Update Input For WBC
    _wbc_data->pBody_des[0] = world_position_desired[0]; //世界系下，期望位置
    _wbc_data->pBody_des[1] = world_position_desired[1];
    _wbc_data->pBody_des[2] = body_height;

    _wbc_data->vBody_des[0] = v_des_world[0];//世界系下，由遥控器输入左乘旋转矩阵
    _wbc_data->vBody_des[1] = v_des_world[1];
    _wbc_data->vBody_des[2] = 0.;

    _wbc_data->aBody_des.setZero();//加速度为0

    _wbc_data->pBody_RPY_des[0] = 0.;
    _wbc_data->pBody_RPY_des[1] = 0.; 
    _wbc_data->pBody_RPY_des[2] = _yaw_des;//_yaw_des = 现在的偏航角（stateEstimator） + dt * _yaw_turn_rate;

    _wbc_data->vBody_Ori_des[0] = 0.;
    _wbc_data->vBody_Ori_des[1] = 0.;
    _wbc_data->vBody_Ori_des[2] = vyaw_rc;//由遥控器输入


    //运动控制
    for(int leg = 0; leg < 4; leg++)
    {
        if(stance_control_flag[leg])//支撑相控制
        {
            if(first_stance[leg])//第一次进入支撑相
            {
                //设置初始位置
                tra[leg].pos_initial = leg_drv->state[leg].p;//当前实际位置
                pb_hf0[leg] = tra[leg].pos_initial;//记录轨迹起点
                first_stance[leg] = false;//更改第一次进入状态
                first_swing[leg]  = true;
            }

            //目标偏航角度------------------------------------------------------------------------论文中表述是与支撑相剩余时间相乘?????????????
            float abd_yaw = 0.5 * vyaw_des * trot_gait.T_stance[leg];
            //期望速度
            Vec3<float> v_des = Vec3<float>(vx_des, vy_des, 0);
            Vec3<float> pb0r_f = ori::coordinateRotation(ori::CoordinateAxis::Z, abd_yaw) * (pb0_bf[leg]); //对应论文中P39(3.78)
            Vec3<float> pb0rt_f = pb0r_f - v_des * (trot_gait.T_stance[leg] - trot_gait.t_stance[leg]);    //对应论文中P39(3.78)

            Vec3<float> p_offset;//p_sy + p_ce
            p_offset(0) = 0.5 * trot_gait.T_stance[leg] * body_data->v_body(0)
                        + 0.08 * (body_data->v_body(0) - v_des(0))
                        + (body_data->position(2)/9.81) * (body_data->v_body(1) * vyaw_des);
            p_offset(1) = 0.5 * trot_gait.T_stance[leg] * body_data->v_body(1)
                        + 0.08 * (body_data->v_body(1) - v_des(1))
                        + (body_data->position(2)/9.81) * (-body_data->v_body(0) * vyaw_des);
            p_offset(2) = 0;//高度方向不做控制
            //足端相对于机身的期望位置
            Vec3<float> pbstd_bf = pb0rt_f - p_offset;

            //足端相对于hip的期望位置
            Vec3<float> pbstd_hf = pbstd_bf - robot::hip_location[leg];
            //目标终点限制
            if(pbstd_hf(0) > robot::stand_fpos[leg](0) + step_max / 2) pbstd_hf(0) = robot::stand_fpos[leg](0) + step_max / 2;
            if(pbstd_hf(0) < robot::stand_fpos[leg](0) - step_max / 2) pbstd_hf(0) = robot::stand_fpos[leg](0) - step_max / 2;
            if(pbstd_hf(1) > robot::stand_fpos[leg](1) + step_max / 2) pbstd_hf(1) = robot::stand_fpos[leg](1) + step_max / 2;
            if(pbstd_hf(1) < robot::stand_fpos[leg](1) - step_max / 2) pbstd_hf(1) = robot::stand_fpos[leg](1) - step_max / 2;
            if(pbstd_hf(2) > robot::stand_fpos[leg](2) + hight_max)    pbstd_hf(2) = robot::stand_fpos[leg](2) + hight_max;
            if(pbstd_hf(2) < robot::stand_fpos[leg](2) - 0.03)         pbstd_hf(2) = robot::stand_fpos[leg](2) - 0.03;

            //设置轨迹的终点
            tra[leg].pos_final = pbstd_hf;
            pb_hf1[leg] = tra[leg].pos_final;//记录轨迹终点

            //执行轨迹规划
            tra[leg].run_st(trot_gait.t_stance[leg] / trot_gait.T_stance[leg], trot_gait.T_stance[leg]);

            // //求足端位置对应的关节角度逆解，并传递给控制指令
            // robot::IK(tra[leg].p, &(leg_drv->cmd[leg].q_des), leg);
            pb_hfc[leg] = tra[leg].p;//记录轨迹当前位置点


            /*为ＷＢＣ足端任务赋值
                @note: 足端位置不动，即将腿基系下的初始足端位置转到世界坐标系，速度加速度均为0，上边的支撑相的轨迹规划无用
            */  
            _wbc_data->pFoot_des[leg] = body_data->position + body_data->r_body.transpose() * (robot::hip_location[leg] + tra[leg].pos_initial);
            _wbc_data->vFoot_des[leg] = Eigen::Matrix<float, 3, 1>::Zero();
            _wbc_data->aFoot_des[leg] = Eigen::Matrix<float, 3, 1>::Zero();
            
            // //足端力控制，足端力转换为关节力矩
            // tau_ff[leg] = leg_drv->state[leg].J.transpose() * f_ff[leg]; //f_ff是mpc解出来的足端反力（世界系），转到机身系下再加个负号，也就是机器人对地面的力。
            
            _wbc_data->Fr_des[leg] = -(body_data->r_body.transpose() * f_ff[leg]);//把f_ff转回世界系

            // for(int joint = 0; joint < 3; joint ++)//最大值限制
            // {
            //     if(tau_ff[leg](joint) >  robot::limit_tau) tau_ff[leg](joint) =  (robot::limit_tau - 1);
            //     if(tau_ff[leg](joint) < -robot::limit_tau) tau_ff[leg](joint) = -(robot::limit_tau - 1);
            // }
            // leg_drv->cmd[leg].tau_ff = tau_ff[leg] * ROBOT_K;

            // leg_drv->cmd[leg].kd = Vec3<float>(0.6, 0.6, 0.6).asDiagonal();

            leg_drv->cmd[leg].kp = Vec3<float>(4, 4, 4).asDiagonal() * ROBOT_K;
            leg_drv->cmd[leg].kd = Vec3<float>(0.4, 0.4, 0.4).asDiagonal() * ROBOT_K;
        }
        
        else//摆动相控制
        {
            if(first_swing[leg])//第一次进入摆动相
            {
                //设置初始位置
                tra[leg].pos_initial = leg_drv->state[leg].p;//当前实际位置
                pb_hf0[leg] = tra[leg].pos_initial;//记录轨迹起点
                first_swing[leg]  = false;//更改第一次进入状态
                first_stance[leg] = true;
            }

            //目标偏航角度-----------------------------------论文中表述是与摆动相剩余时间相乘，0.5可以理解成缩小指令?????????????
            float abd_yaw = 0.5 * vyaw_des * trot_gait.T_stance[leg];
            //期望平动速度
            Vec3<float> v_des = Vec3<float>(vx_des, vy_des, 0);
            Vec3<float> pb0r_f = ori::coordinateRotation(ori::CoordinateAxis::Z, -abd_yaw) * (pb0_bf[leg]); //对应论文中P29(3.14)
            Vec3<float> pb0rt_f = pb0r_f + v_des * (trot_gait.T_swing[leg] - trot_gait.t_swing[leg]); //对应论文中P29(3.14)

            Vec3<float> p_offset; //p_sy + p_ce
            p_offset(0) = 0.5 * trot_gait.T_stance[leg] * body_data->v_body(0)
                        + 0.08 * (body_data->v_body(0) - v_des(0))                           //对应论文中P30(3.15)
                        + (body_data->position(2)/9.81) * (body_data->v_body(1) * vyaw_des); //对应论文中P31(3.21)----这里没用机身真实角速度
            p_offset(1) = 0.5 * trot_gait.T_stance[leg] * body_data->v_body(1)
                        + 0.08 * (body_data->v_body(1) - v_des(1))                            //对应论文中P30(3.15)
                        + (body_data->position(2)/9.81) * (-body_data->v_body(0) * vyaw_des); //对应论文中P31(3.21)
            p_offset(2) = 0; //高度方向不做控制
            Vec3<float> pbswd_bf = pb0rt_f + p_offset;

            //足端相对于hip的期望位置
            Vec3<float> pbswd_hf = pbswd_bf - robot::hip_location[leg];
            //落足点限制
            if(pbswd_hf(0) > robot::stand_fpos[leg](0) + step_max / 2) pbswd_hf(0) = robot::stand_fpos[leg](0) + step_max / 2;
            if(pbswd_hf(0) < robot::stand_fpos[leg](0) - step_max / 2) pbswd_hf(0) = robot::stand_fpos[leg](0) - step_max / 2;
            if(pbswd_hf(1) > robot::stand_fpos[leg](1) + step_max / 2) pbswd_hf(1) = robot::stand_fpos[leg](1) + step_max / 2;
            if(pbswd_hf(1) < robot::stand_fpos[leg](1) - step_max / 2) pbswd_hf(1) = robot::stand_fpos[leg](1) - step_max / 2;
            if(pbswd_hf(2) > robot::stand_fpos[leg](2) + hight_max)    pbswd_hf(2) = robot::stand_fpos[leg](2) + hight_max;
            if(pbswd_hf(2) < robot::stand_fpos[leg](2) - 0.03)         pbswd_hf(2) = robot::stand_fpos[leg](2) - 0.03;
            
            //抬腿高度限制
            if(tra[leg].pos_initial(2) + hf_des > robot::stand_fpos[leg](2) + hight_max) 
                hf_des = robot::stand_fpos[leg](2) + hight_max - tra[leg].pos_initial(2);
            //设置摆动高度
            tra[leg].height = hf_des;
            //设置轨迹的终点,在腿的坐标系下
            tra[leg].pos_final = pbswd_hf;
            pb_hf1[leg] = tra[leg].pos_final; //记录轨迹终点

            /*执行摆动相轨迹规划
            @input: (在腿坐标系下的足端)起始点、终止点、抬腿高度、(摆动相)总时间、现在时间
            @output: (现在时间的)p、v、a
            */
            tra[leg].run_sw(trot_gait.t_swing[leg] / trot_gait.T_swing[leg], trot_gait.T_swing[leg]);
            
            // //求足端位置对应的关节角度逆解，并传递给控制指令
            // robot::IK(tra[leg].p, &(leg_drv->cmd[leg].q_des), leg);
            pb_hfc[leg] = tra[leg].p;//记录轨迹当前位置点

            /*为ＷＢＣ足端任务赋值
            @note:规划在腿基系下的足端位置、速度、加速度转到世界坐标系
            */
            _wbc_data->pFoot_des[leg] = body_data->position + body_data->r_body.transpose() * (robot::hip_location[leg] + tra[leg].p);
            _wbc_data->vFoot_des[leg] = body_data->r_body.transpose() * tra[leg].v;//世界系下                                 
            _wbc_data->aFoot_des[leg] = body_data->r_body.transpose() * tra[leg].a; 
            //启动该腿的关节位置PD控制
#ifdef ROBOT1
            leg_drv->cmd[leg].kp = Vec3<float>(20, 20, 20).asDiagonal() * ROBOT_K;
            leg_drv->cmd[leg].kd = Vec3<float>(0.2, 0.2, 0.2).asDiagonal() * ROBOT_K;
#endif

#ifdef ROBOT2
            leg_drv->cmd[leg].kp = Vec3<float>(40, 20, 20).asDiagonal() * ROBOT_K;
            leg_drv->cmd[leg].kd = Vec3<float>(0.4, 0.2, 0.2).asDiagonal() * ROBOT_K;
#endif
            // leg_drv->cmd[leg].kp = Vec3<float>(40, 40, 40).asDiagonal();
            // leg_drv->cmd[leg].kd = Vec3<float>(0.4, 0.4, 0.4).asDiagonal();
            // leg_drv->cmd[leg].kp = Vec3<float>(30, 30, 30).asDiagonal();
            // leg_drv->cmd[leg].kd = Vec3<float>(0.3, 0.3, 0.3).asDiagonal();
        }
    }

    //　ＷＢＣ要的支撑状态
    for(int leg(0); leg < 4; leg++){
        _wbc_data->contact_state[leg] = stance_control_flag[leg];
    }

    //运行WBC
    _wbc_ctrl->WBC_run(_wbc_data);

    for(size_t leg(0); leg < (size_t)4; ++leg) //根据WBC结果更新指令
    {
        for(int joint = 0; joint < 3; joint ++)//最大值限制
        {
            if(_wbc_ctrl->_WBC_legcmd[leg].tau_ff(joint) >  robot::limit_tau) 
                _wbc_ctrl->_WBC_legcmd[leg].tau_ff(joint) =  (robot::limit_tau - 1);
            if(_wbc_ctrl->_WBC_legcmd[leg].tau_ff(joint) < -robot::limit_tau) 
                _wbc_ctrl->_WBC_legcmd[leg].tau_ff(joint) = -(robot::limit_tau - 1);
        }
        
        leg_drv->cmd[leg].tau_ff = _wbc_ctrl->_WBC_legcmd[leg].tau_ff * ROBOT_K;
        leg_drv->cmd[leg].q_des = _wbc_ctrl->_WBC_legcmd[leg].q_des;
        leg_drv->cmd[leg].qd_des = _wbc_ctrl->_WBC_legcmd[leg].qd_des;
        //kp,kd交给运动控制更新
    }    
}
#endif /* WBCANDMPC */

#ifdef ONLYMPC
/**
 * @description: 运动控制
 * @param {无}
 * @return {无}
 */
void locomotion_controller::run()
{
    trot_gait.run();//执行步态规划，每个控制周期都要执行一次
    
    //目前已经在leg_controller中更新为足端接触状态
    for(int leg = 0; leg < 4; leg++)
    {
        //更新状态估计中的足端接触状态数据
        body_data->contact_state(leg) = trot_gait.stance_state[leg];
    }
    
#ifndef FROBOT1 //只要不是1做从机（包含直接遥控1和2的情况），便使用遥控器数据
    //遥控器期望前后方向速度指令
    //遥控器期望左右方向速度指令
    //遥控器期望偏航角速度指令
    //遥控器期望抬腿高度指令
    float vx_rc   = rc_cmd->RCO * vx_max;
    float vy_rc   = rc_cmd->LRO * vy_max;
    float vyaw_rc = rc_cmd->RRO * vyaw_max;
    float hf_rc   = rc_cmd->VB  * hight_max;
#endif

#ifdef FROBOT1 //只要1做从机，将指令替换为从以下策略获得
    // 平行跟随策略
    float vx_rc   = 0.f;
    float vy_rc   = 0.f;
    float vyaw_rc = rc_cmd->RRO * vyaw_max;
    float hf_rc   = rc_cmd->VB  * hight_max;
    if(1 == 1){
        float vx_kp = 3.;
        float vx_kd = 0.5;
        
        float vx_adjustment = vx_kp * (usb_data->sdata[2] * M_PI / 180.0f) - vx_kd * body_data->v_body(0);
        vx_rc += vx_adjustment;

        const float max_vx = 0.6f;   // 限制最大前进速度
        vx_rc = std::min(max_vx, vx_rc);
        const float min_vx = 0.f;    // 限制最小前进速度
        vx_rc = std::max(min_vx, vx_rc); 
    }
#endif
    
    float filter = 0.005; //指令滤波系数0.005，一阶低通滤波，系数越小滤波强响应慢
    float filter_hf = 0.03;

    vx_des   = vx_rc   * filter + vx_des   * (1 - filter);
    vy_des   = vy_rc   * filter + vy_des   * (1 - filter);
    vyaw_des = vyaw_rc * filter + vyaw_des * (1 - filter);
    hf_des   = hf_rc   * filter_hf   + hf_des * (1 - filter_hf);

    for(int leg = 0; leg < 4; leg++)//基于实际触地状态的复杂地形运动控制策略
    {
        if(trot_gait.stance_flag[leg])//规划的支撑相
        {
            stance_control_flag[leg] = true;//支撑控制
        }
        else//规划的摆动相
        {
            if(trot_gait.t_swing[leg] < 0.5)//前半段
            {
                stance_control_flag[leg] = false;//摆动控制
            }
            else//后半段
            {
                if(contact_flag[leg])//实际已经触地
                {
                    stance_control_flag[leg] = true;//支撑控制
                }
                else//实际未触地
                {
                    stance_control_flag[leg] = false;//摆动控制
                }
            }
        }
    }
    
    //MPC计算足端力,每过一个dtmpc/dt_gait，执行一次run_MPC()
    if(trot_gait.c == 1) run_MPC();

    //运动控制
    for(int leg = 0; leg < 4; leg++)
    {
        if(stance_control_flag[leg])//支撑相控制
        {
            if(first_stance[leg])//第一次进入支撑相
            {
                //设置初始位置
                tra[leg].pos_initial = leg_drv->state[leg].p;//当前实际位置
                pb_hf0[leg] = tra[leg].pos_initial;//记录轨迹起点
                first_stance[leg] = false;//更改第一次进入状态
                first_swing[leg]  = true;
            }

            //目标偏航角度
            float abd_yaw = 0.5 * vyaw_des * trot_gait.T_stance[leg];
            //期望速度
            Vec3<float> v_des = Vec3<float>(vx_des, vy_des, 0);
            Vec3<float> pb0r_f = ori::coordinateRotation(ori::CoordinateAxis::Z, abd_yaw) * (pb0_bf[leg]);
            Vec3<float> pb0rt_f = pb0r_f - v_des * (trot_gait.T_stance[leg] - trot_gait.t_stance[leg]);

            Vec3<float> p_offset;
            p_offset(0) = 0.5 * trot_gait.T_stance[leg] * body_data->v_body(0)
                        + 0.08 * (body_data->v_body(0) - v_des(0))
                        + (body_data->position(2)/9.81) * (body_data->v_body(1) * vyaw_des);
            p_offset(1) = 0.5 * trot_gait.T_stance[leg] * body_data->v_body(1)
                        + 0.08 * (body_data->v_body(1) - v_des(1))
                        + (body_data->position(2)/9.81) * (-body_data->v_body(0) * vyaw_des);
            p_offset(2) = 0;//高度方向不做控制
            //足端相对于机身的期望位置
            Vec3<float> pbstd_bf = pb0rt_f - p_offset;

            //足端相对于hip的期望位置
            Vec3<float> pbstd_hf = pbstd_bf - robot::hip_location[leg];
            //目标终点限制
            if(pbstd_hf(0) > robot::stand_fpos[leg](0) + step_max / 2) pbstd_hf(0) = robot::stand_fpos[leg](0) + step_max / 2;
            if(pbstd_hf(0) < robot::stand_fpos[leg](0) - step_max / 2) pbstd_hf(0) = robot::stand_fpos[leg](0) - step_max / 2;
            if(pbstd_hf(1) > robot::stand_fpos[leg](1) + step_max / 2) pbstd_hf(1) = robot::stand_fpos[leg](1) + step_max / 2;
            if(pbstd_hf(1) < robot::stand_fpos[leg](1) - step_max / 2) pbstd_hf(1) = robot::stand_fpos[leg](1) - step_max / 2;
            if(pbstd_hf(2) > robot::stand_fpos[leg](2) + hight_max)    pbstd_hf(2) = robot::stand_fpos[leg](2) + hight_max;
            if(pbstd_hf(2) < robot::stand_fpos[leg](2) - 0.03)         pbstd_hf(2) = robot::stand_fpos[leg](2) - 0.03;

            //设置轨迹的终点
            tra[leg].pos_final = pbstd_hf;
            pb_hf1[leg] = tra[leg].pos_final;//记录轨迹终点

            //执行轨迹规划
            tra[leg].run_st(trot_gait.t_stance[leg] / trot_gait.T_stance[leg], trot_gait.T_stance[leg]);

            //求足端位置对应的关节角度逆解，并传递给控制指令
            robot::IK(tra[leg].p, &(leg_drv->cmd[leg].q_des), leg);
            pb_hfc[leg] = tra[leg].p;//记录轨迹当前位置点
            
            //足端力控制，足端力转换为关节力矩
            tau_ff[leg] = leg_drv->state[leg].J.transpose() * f_ff[leg];
            for(int joint = 0; joint < 3; joint ++)//最大值限制
            {
                if(tau_ff[leg](joint) >  robot::limit_tau) tau_ff[leg](joint) =  (robot::limit_tau - 1);
                if(tau_ff[leg](joint) < -robot::limit_tau) tau_ff[leg](joint) = -(robot::limit_tau - 1);
            }
            leg_drv->cmd[leg].tau_ff = tau_ff[leg] * ROBOT_K;

            // leg_drv->cmd[leg].kd = Vec3<float>(0.6, 0.6, 0.6).asDiagonal();

            leg_drv->cmd[leg].kp = Vec3<float>(4, 4, 4).asDiagonal() * ROBOT_K;
            leg_drv->cmd[leg].kd = Vec3<float>(0.4, 0.4, 0.4).asDiagonal() * ROBOT_K;
        }
        else//摆动相控制
        {
            if(first_swing[leg])//第一次进入摆动相
            {
                //设置初始位置
                tra[leg].pos_initial = leg_drv->state[leg].p;//当前实际位置
                pb_hf0[leg] = tra[leg].pos_initial;//记录轨迹起点
                first_swing[leg]  = false;//更改第一次进入状态
                first_stance[leg] = true;
            }

            //目标偏航角度
            float abd_yaw = 0.5 * vyaw_des * trot_gait.T_stance[leg];
            //期望速度
            Vec3<float> v_des = Vec3<float>(vx_des, vy_des, 0);
            Vec3<float> pb0r_f = ori::coordinateRotation(ori::CoordinateAxis::Z, -abd_yaw) * (pb0_bf[leg]);
            Vec3<float> pb0rt_f = pb0r_f + v_des * (trot_gait.T_swing[leg] - trot_gait.t_swing[leg]);

            Vec3<float> p_offset;
            p_offset(0) = 0.5 * trot_gait.T_stance[leg] * body_data->v_body(0)
                        + 0.08 * (body_data->v_body(0) - v_des(0))
                        + (body_data->position(2)/9.81) * (body_data->v_body(1) * vyaw_des);
            p_offset(1) = 0.5 * trot_gait.T_stance[leg] * body_data->v_body(1)
                        + 0.08 * (body_data->v_body(1) - v_des(1))
                        + (body_data->position(2)/9.81) * (-body_data->v_body(0) * vyaw_des);
            p_offset(2) = 0;//高度方向不做控制
            Vec3<float> pbswd_bf = pb0rt_f + p_offset;

            //足端相对于hip的期望位置
            Vec3<float> pbswd_hf = pbswd_bf - robot::hip_location[leg];
            //目标终点限制
            if(pbswd_hf(0) > robot::stand_fpos[leg](0) + step_max / 2) pbswd_hf(0) = robot::stand_fpos[leg](0) + step_max / 2;
            if(pbswd_hf(0) < robot::stand_fpos[leg](0) - step_max / 2) pbswd_hf(0) = robot::stand_fpos[leg](0) - step_max / 2;
            if(pbswd_hf(1) > robot::stand_fpos[leg](1) + step_max / 2) pbswd_hf(1) = robot::stand_fpos[leg](1) + step_max / 2;
            if(pbswd_hf(1) < robot::stand_fpos[leg](1) - step_max / 2) pbswd_hf(1) = robot::stand_fpos[leg](1) - step_max / 2;
            if(pbswd_hf(2) > robot::stand_fpos[leg](2) + hight_max)    pbswd_hf(2) = robot::stand_fpos[leg](2) + hight_max;
            if(pbswd_hf(2) < robot::stand_fpos[leg](2) - 0.03)         pbswd_hf(2) = robot::stand_fpos[leg](2) - 0.03;
            //抬腿高度限制
            if(tra[leg].pos_initial(2) + hf_des > robot::stand_fpos[leg](2) + hight_max) hf_des = robot::stand_fpos[leg](2) + hight_max - tra[leg].pos_initial(2);
            //设置摆动高度
            tra[leg].height = hf_des;
            //设置轨迹的终点
            tra[leg].pos_final = pbswd_hf;
            pb_hf1[leg] = tra[leg].pos_final;//记录轨迹终点

            //执行轨迹规划
            tra[leg].run_sw(trot_gait.t_swing[leg] / trot_gait.T_swing[leg], trot_gait.T_swing[leg]);

            //求足端位置对应的关节角度逆解，并传递给控制指令
            robot::IK(tra[leg].p, &(leg_drv->cmd[leg].q_des), leg);
            pb_hfc[leg] = tra[leg].p;//记录轨迹当前位置点

            //启动该腿的关节位置PD控制
#ifdef ROBOT1
            leg_drv->cmd[leg].kp = Vec3<float>(20, 20, 20).asDiagonal() * ROBOT_K;
            leg_drv->cmd[leg].kd = Vec3<float>(0.2, 0.2, 0.2).asDiagonal() * ROBOT_K;
#endif

#ifdef ROBOT2
            leg_drv->cmd[leg].kp = Vec3<float>(40, 20, 20).asDiagonal() * ROBOT_K;
            leg_drv->cmd[leg].kd = Vec3<float>(0.4, 0.2, 0.2).asDiagonal() * ROBOT_K;
#endif
            // leg_drv->cmd[leg].kp = Vec3<float>(40, 40, 40).asDiagonal();
            // leg_drv->cmd[leg].kd = Vec3<float>(0.4, 0.4, 0.4).asDiagonal();
            // leg_drv->cmd[leg].kp = Vec3<float>(30, 30, 30).asDiagonal();
            // leg_drv->cmd[leg].kd = Vec3<float>(0.3, 0.3, 0.3).asDiagonal();
        }
    }
}
#endif /* ONLYMPC */


/**
 * @description: 计算mpc，求解足端力
 * @param {无}
 * @return {无}
 */
void locomotion_controller::run_MPC()
{
    Timer run_mpc_dt;
    
    //mpc一次预测的所有腿支撑状态，1：支撑，0：摆动
    int mpc_stance_flag[4*mpc_step];
    for(int leg = 0; leg < 4; leg++)//第一个预测步长的支撑状态就是当前状态
    {
        mpc_stance_flag[leg] = stance_control_flag[leg];
    }
    for(int i = 1; i < mpc_step; i++)//后续预测步长
    {
        //计算当前步支撑状态
        trot_gait.run_mpc_stance_flag(trot_gait.t + dtmpc * i);
        for(int leg = 0; leg < 4; leg++)//按腿复制支撑状态
        {
            mpc_stance_flag[4 * i + leg] = trot_gait.mpc_stance_flag[leg];
        }
    }
    
    //机器人坐标系下期望速度
    //世界坐标系下期望速度
    Vec3<float> v_des_robot(vx_des, vy_des,0);
    Vec3<float> v_des_world = body_data->r_body.transpose() * v_des_robot;

    //初始参考轨迹-机身xy位置和偏航角
    float px = dtmpc * v_des_world(0) + body_data->position(0);
    float py = dtmpc * v_des_world(1) + body_data->position(1);
    float yaw_des = dtmpc * vyaw_des + body_data->rpy(2);
    //初始参考轨迹
    float trajInitial[12] = 
    {
        0,  0,  yaw_des,                  //角度
        px, py, body_height,              //位置
        0,  0,  vyaw_des,                 //角速度
        v_des_world(0), v_des_world(1), 0 //速度
    };
    //MPC完整轨迹
    float mpc_traj[12*mpc_step];
    for(int i = 0; i < mpc_step; i++)
    {
        //每个步长预先存入初始轨迹
        for(int j = 0; j < 12; j++) mpc_traj[12*i+j] = trajInitial[j];
        if(i > 0)//第一个预测步长为初始轨迹，故不修改，后面步长按时间和速度修改
        {
            //后续预测轨迹
            mpc_traj[12*i + 2] = mpc_traj[12 * (i - 1) + 2] + dtmpc * vyaw_des;
            mpc_traj[12*i + 3] = mpc_traj[12 * (i - 1) + 3] + dtmpc * v_des_world(0);
            mpc_traj[12*i + 4] = mpc_traj[12 * (i - 1) + 4] + dtmpc * v_des_world(1);
        }
    }

    //权重
    // float Q[12] = {0.25, 0.25, 10, 2, 2, 40, 0, 0, 0.3, 0.2, 0.2, 0.2};
    // float Q[12] = {25, 25, 10, 1, 1, 50, 0, 0, 0.3, 0.2, 0.2, 0.1};  
    // float Q[12] = {20, 20, 10, //角度
    //                2, 2, 40, //位置
    //                0, 0, 0.3, //角速度
    //                0.2, 0.2, 0.1};//速度
      
    // float Q[12] = {20, 20, 10, 5, 5, 10, 0, 0, 0.3, 0.3, 0.3, 0.1};  
    
#if defined(ROBOT1) && defined(FROBOT1)
    float Q[18] = {20, 20, 10, 20, 20, 10, 0, 0, 0.3, 0.3, 0.3, 0.1, 0, 0, 0, 0, 0, 0}; // 考虑外力，18列
#endif
#if defined(ROBOT1) && (!defined(FROBOT1)) // 1号机遥控Q阵
    float Q[15] = {20, 20, 10, 20, 20, 10, 0, 0, 0.3, 0.3, 0.3, 0.1, 0, 0, 0}; 
#endif
#ifdef ROBOT2                              // 2号机遥控Q阵
    float Q[15] = {10, 10, 5, 20, 40, 10, 0, 0, 0.3, 0.3, 0.3, 0.1, 0, 0, 0};
#endif
    
    //mpc参数
    float yaw = body_data->rpy(2);
    float* weights = Q; //二次优化标准表达式中Q矩阵的左上角12x12子阵主对角线元素的值
    float alpha = 4e-5; //二次优化标准表达式中R矩阵的系数值
    // float alpha = 1e-6;
    float p[3] = {body_data->position(0),    body_data->position(1),    body_data->position(2)};
    float v[3] = {body_data->v_world(0),     body_data->v_world(1),     body_data->v_world(2)};
    float w[3] = {body_data->omega_world(0), body_data->omega_world(1), body_data->omega_world(2)};
    float q[4] = {body_data->orientation(0), body_data->orientation(1), body_data->orientation(2), body_data->orientation(3)};

#if defined(ROBOT1) && defined(FROBOT1) //获取从机上的力传感的数据输入MPC
    
    float Fex[3] = {0., 0., 0.};

    //外力向量，注意要转到世界系下
    Vec3<float> Fex_vec(usb_data->sdata[3], 
                        usb_data->sdata[4], 
                        usb_data->sdata[5]);
    Fex_vec = body_data->r_body.transpose() * Fex_vec; //转到世界系下
    for(int i = 0; i < 3; i++){
        Fex[i] = Fex_vec(i);
    }

#endif

    float r[12];//世界系下足端位置
    for(int leg = 0; leg < 4; leg++)
    {
        Vec3<float> pf = body_data->r_body.transpose() * (robot::hip_location[leg] + leg_drv->state[leg].p);
        for(int i = 0; i < 3; i++)
        {
            // r[leg*3 + i] = pf(i);//大bug
            r[i*4+leg] = pf(i);  //[x,x,x,x,y,y,y,y,z,z,z,z]
        }
    }

    //  setup_problem(dtmpc,mpc_step,0.4,150);

    // update_x_drag(x_comp_integral);//x方向速度对z轴方向位置偏差的影响
    // if(v[0] > 0.3 || v[0] < -0.3)
    // {
    //     float pz_err = p[2] - body_height;
    //     x_comp_integral += 3 * pz_err * dtmpc / v[0];
    // }

    // update_solver_settings(10000, 1e-07, 1e-08, 1.5, 0.1, 0);

#if defined(ROBOT1) && defined(FROBOT1) //只要1做从机，有外力MPC
    update_Fex_problem_data_floats(p,v,q,w,Fex,r,yaw,weights,mpc_traj,alpha,mpc_stance_flag); //带外力的MPC，更新求解MPC的参数，并求解MPC
#endif

#ifndef FROBOT1 //只要不是1做从机（包含直接遥控1和2的情况），无外力MPC
    update_problem_data_floats(p,v,q,w,r,yaw,weights,mpc_traj,alpha,mpc_stance_flag); //没有考虑外力的MPC，更新求解MPC的参数，并求解MPC        
#endif
    

    //获取mpc结果-足端力
    for(int leg = 0; leg < 4; leg++)
    {
        Vec3<float> f;
        for(int i = 0; i < 3; i++) f(i) = get_solution(leg*3 + i);
        f_ff[leg] = -body_data->r_body * f;//f是mpc解出来的足端反力（世界系），转到机身系下再加个负号，也就是机器人对地面的力f_ff。
    }

    mpc_real_dt = run_mpc_dt.getMs();//记录执行一次MPC的真实时间

}


/*
    WBC function call ref_code
*/

// WBC_Ctrl<T> *_wbc_ctrl;
// LocomotionCtrlData<T> * _wbc_data;
// _wbc_ctrl = new LocomotionCtrl<T>(dynamic, body_data, leg_drv->state);


// Vec3<float> v_des_world = body_data.r_body.transpose() * Vec3<float>(vx_des, vy_des, 0);
// Vec3<float> world_position_desired = body_data.position + robot::dt * v_des_world;
// float _yaw_des = body_data.rpy[2] + robot::dt * vyaw_rc;

// // Update Input For WBC
// _wbc_data->pBody_des[0] = world_position_desired[0]; //世界系下，world_position_desired += dt * Vec3<float>(v_des_world[0], v_des_world[1], 0);
// _wbc_data->pBody_des[1] = world_position_desired[1];
// _wbc_data->pBody_des[2] = body_height;

// _wbc_data->vBody_des[0] = v_des_world[0];//世界系下，由遥控器输入左乘旋转矩阵
// _wbc_data->vBody_des[1] = v_des_world[1];
// _wbc_data->vBody_des[2] = 0.;

// _wbc_data->aBody_des.setZero();//加速度为0

// _wbc_data->pBody_RPY_des[0] = 0.;
// _wbc_data->pBody_RPY_des[1] = 0.; 
// _wbc_data->pBody_RPY_des[2] = _yaw_des;//_yaw_des = 现在的偏航角（stateEstimator） + dt * _yaw_turn_rate;

// _wbc_data->vBody_Ori_des[0] = 0.;
// _wbc_data->vBody_Ori_des[1] = 0.;
// _wbc_data->vBody_Ori_des[2] = vyaw_rc;//由遥控器输入

// for(int foot(0); foot < 4; foot++){
//     if(/*foot是摆动腿*/){
        
//         //在腿基系下的足端位置转到世界坐标系
//         Vec3<float> pDesFootWorld = body_data->position + body_data->r_body.transpose() * (robot::hip_location[leg] + tra[leg].p);
//         Vec3<float> vDesFootWorld = body_data->r_body.transpose() * tra[leg].v;
//         Vec3<float> aDesFootWorld = body_data->r_body.transpose() * tra[leg].a;

//         _wbc_data->pFoot_des[foot] = pDesFootWorld;//世界系下
//         _wbc_data->vFoot_des[foot] = vDesFootWorld;                                 
//         _wbc_data->aFoot_des[foot] = footSwingTrajectories[foot].getAcceleration(); 
//     }else{
//         //（MIT方案）不进行WBC输入的更新        
//         //（猜测）对于支撑足来说，pFoot_des保持不变，v和a设为0

//         _wbc_data->Fr_des[foot] = /*MPC计算的力*/;
//     }
// }

// _wbc_data->contact_state = gait->getContactState();/*按照gait_plan中MPC规划的状态来*/

// //运行WBC
// _wbc_ctrl->WBC_run(_wbc_data);


// for(size_t leg(0); leg < (size_t)4; ++leg){ //根据WBC结果更新指令
//     leg_drv->cmd[leg].tau_ff = _wbc_ctrl->_WBC_legcmd[leg].tau_ff;
//     leg_drv->cmd[leg].q_des = _wbc_ctrl->_WBC_legcmd[leg].q_des;
//     leg_drv->cmd[leg].qd_des = _wbc_ctrl->_WBC_legcmd[leg].qd_des;
//     //kp,kd交给运动控制更新
// }
