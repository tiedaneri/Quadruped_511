/*
 * @Author: sun
 * @Date: 2021-01-20 09:16:35
 * @LastEditTime: 2025-09-24 11:02:09
 * @LastEditors: Please set LastEditors
 * @Description: 腿控制器源文件
 * @FilePath: /robot_software/user/leg_controller/leg_controller.cpp
 */

#include "leg_controller.h"
#include <Utilities/Timer.h>//记录程序运行时间

/**
 * @description: 构造函数
 * @param {无}
 * @return {无}
 */
leg_controller::leg_controller()
{

}

/**
 * @description: 析构函数，删除对象
 * @param {无}
 * @return {无}
 */
leg_controller::~leg_controller()
{
    delete mode_ctr;//删除模式控制对象
    delete safty_ck;//删除安全检查对象

    fclose(data_file);//关闭文件

    printf("[leg_controller] leg_controller ended\n");
}

/**
 * @description: 初始化各参数
 * @param {无}
 * @return {无}
 */
void leg_controller::init()
{
    model.build();//创建运动树模型    
    //创建浮动基动力学计算对象，并传递对象参数
    dynamic = new fb_dynamic(&model, leg_drv->state, body_data);
    dynamic->init();//初始化动力学
    //创建模式控制对象，并传递对象参数
    mode_ctr = new mode_controller(&rc_cmd, leg_drv, body_data, dynamic, &usb_data);
    mode_ctr->init();//初始化模式控制器
    //创建安全检查对象，并传递对象参数
    safty_ck = new safety_check(leg_drv, &rc_cmd, mode_ctr, body_data);
    //创建接触状态检查对象，传递参数
    contact = new contact_detection(mode_ctr->locomotion_ctr->trot_gait.stance_flag,
                                    mode_ctr->locomotion_ctr->trot_gait.stance_state,
                                    mode_ctr->locomotion_ctr->trot_gait.swing_state,
                                    dynamic->Fc);

    real_t = 0;
    ctr_dt = 0;

    dynamic->time_C = 0;
    dynamic->time_H = 0;

    data_file = fopen("robot_data.txt", "w");//打开保存数据的文件
}

/**
 * @description: 控制器主程序，循环运行
 * @param {无}
 * @return {无}
 */
void leg_controller::run_controller()
{
    Timer controller_dt;

    safty_ck->run_state_check();//运行机器人状态检查
    dynamic->run_fd();//正动力学计算合外力
    dynamic->run_cf();//计算足端接触力
    
#ifdef WBCANDMPC
    dynamic->WBC_kin();
    dynamic->WBC_TaskJ();//WBC计算任务雅各比和JdotQdot
#endif /* WBCANDMPC */

    dynamic_real_dt = controller_dt.getMs();//记录动力学运行一次的真实时间

    contact->run();//计算足端接触状态
    // for(int leg = 0; leg < 4; leg++)
    // {
    //     //将接触状态传递给body_data
    //     body_data->contact_state(leg) = contact->pc[leg];
    // }

    for(int leg = 0; leg < 4; leg++)
    {
        //将接触状态传递给locomotion_ctr
        mode_ctr->locomotion_ctr->contact_flag[leg] = contact->flag[leg];
    }

    mode_ctr->set_mode();//设置控制模式
    mode_ctr->run_mode();//运行设定模式
    safty_ck->run_cmd_check();//运行指令检查

    // run_data_write();//执行将各种数据写到文件中--（终端/文件打印数据开关）

    real_t += robot::dt; //更新实时时间

    ctr_dt = controller_dt.getMs();//记录控制器运行一次的时间
}

/**
 * @description: 执行将各种数据写到文件中
 * @param {无}
 * @return {无}
 */
void leg_controller::run_data_write()
{
    static float i = 0;//记录时间间隔
    static bool error_write = false;//出错保存数据标志
    if(safty_ck->state_check_code > 0 || safty_ck->cmd_check_code > 0)//发生错误
    {
        if(error_write == false)//发生错误未保存数据
        {
            write_robot_data();//保存数据
            error_write = true;//更新标志
        }
    }
    // if(i >= 0.02)//20ms保存一次数据
    // if(i >= 0.004)//4ms保存一次数据
    if(i >= 0.1)//100ms保存一次数据
    {
        write_robot_data();
        i = 0;
    }
    else
    {
        i += robot::dt;
    }
    
}

/**
 * @description: 将各种数据写到文件中
 * @param {无}
 * @return {无}
 */
void leg_controller::write_robot_data()
{
    // if(mode_ctr->mode == locomotion) data_test1();//进入运动模式时才保存数据
    // if(mode_ctr->mode == locomotion) data_test2();//进入运动模式时才保存数据
    // if(mode_ctr->mode == locomotion) data_test3();//进入运动模式时才保存数据
    data_all();
}
/**
 * @description: 复杂地形运动数据
 * @param {无}
 * @return {无}
 */
void leg_controller::data_test3()
{
    //第1列，实时时间
    fprintf(data_file, "%.3f ", real_t);
    

    fprintf(data_file, "%.3f %.3f %.3f ", //只打印：姿态欧拉角，世界坐标系下，机身相对于世界坐标系
                        body_data->rpy(0),
                        body_data->rpy(1),
                        body_data->rpy(2));
    // //共3*9列，机身状态估计数据
    // fprintf(data_file, "%.3f %.3f %.3f ", //姿态欧拉角，世界坐标系下，机身相对于世界坐标系
    //                     body_data->rpy(0),
    //                     body_data->rpy(1),
    //                     body_data->rpy(2));
    // fprintf(data_file, "%.3f %.3f %.3f ", //机身速度，机身坐标系下，机身相对于世界坐标系
    //                     body_data->v_body(0),
    //                     body_data->v_body(1),
    //                     body_data->v_body(2));
    // fprintf(data_file, "%.3f %.3f %.3f ", //机身相对于世界坐标系的位置在世界坐标系下的表示
    //                     body_data->position(0),
    //                     body_data->position(1),
    //                     body_data->position(2));
    // fprintf(data_file, "%.3f %.3f %.3f ", //机身加速度，机身坐标系下，机身相对于世界坐标系
    //                     body_data->a_body(0),
    //                     body_data->a_body(1),
    //                     body_data->a_body(2)); 
    fprintf(data_file, "\n");//结束，换行                 
}

/**
 * @description: 足端触底状态估计数据
 * @param {无}
 * @return {无}
 */
void leg_controller::data_test2()
{
    //第1列，实时时间
    fprintf(data_file, "%.3f ", real_t);
    //共4列，支撑标志
    for(int leg = 0; leg < 4; leg++)
    {
        fprintf(data_file, "%d ", mode_ctr->locomotion_ctr->trot_gait.stance_flag[leg]);
    }
    //共4列，支撑状态
    for(int leg = 0; leg < 4; leg++)
    {
        fprintf(data_file, "%.3f ", mode_ctr->locomotion_ctr->trot_gait.stance_state[leg]);
    }
    //共4列，摆动状态
    for(int leg = 0; leg < 4; leg++)
    {
        fprintf(data_file, "%.3f ", mode_ctr->locomotion_ctr->trot_gait.swing_state[leg]);
    }
    //共4列，预测概率
    for(int leg = 0; leg < 4; leg++)
    {
        fprintf(data_file, "%.3f ", contact->pp[leg]);
    }
    //共4列，足端z向接触力
    for(int leg = 0; leg < 4; leg++)
    {
        fprintf(data_file, "%.3f ", dynamic->Fc[leg](2));
    }
    //共4列，测量概率
    for(int leg = 0; leg < 4; leg++)
    {
        fprintf(data_file, "%.3f ", contact->pm[leg]);
    }       
    //共4列，融合概率
    for(int leg = 0; leg < 4; leg++)
    {
        fprintf(data_file, "%.3f ", contact->pc[leg]);
    }  
    fprintf(data_file, "\n");//结束，换行                 
}

/**
 * @description: 力位混合测试数据
 * @param {无}
 * @return {无}
 */
void leg_controller::data_test1()
{
    //第1列，实时时间
    fprintf(data_file, "%.3f ", real_t);
    //共3*9列，机身状态估计数据
    fprintf(data_file, "%.3f %.3f %.3f ", //姿态欧拉角，世界坐标系下，机身相对于世界坐标系
                        body_data->rpy(0),
                        body_data->rpy(1),
                        body_data->rpy(2));
    fprintf(data_file, "%.3f %.3f %.3f ", //机身速度，机身坐标系下，机身相对于世界坐标系
                        body_data->v_body(0),
                        body_data->v_body(1),
                        body_data->v_body(2));
    fprintf(data_file, "%.3f %.3f %.3f ", //机身相对于世界坐标系的位置在世界坐标系下的表示
                        body_data->position(0),
                        body_data->position(1),
                        body_data->position(2));
    fprintf(data_file, "%.3f %.3f %.3f ", //机身加速度，机身坐标系下，机身相对于世界坐标系
                        body_data->a_body(0),
                        body_data->a_body(1),
                        body_data->a_body(2));
    for(int leg = 0; leg < 4; leg++)
    {
        fprintf(data_file, "%.3f %.3f %.3f ", //足端位置
                    leg_drv->state[leg].p(0),
                    leg_drv->state[leg].p(1),
                    leg_drv->state[leg].p(2));
    }  
    for(int leg = 0; leg < 4; leg++)
    {
        fprintf(data_file, "%.3f %.3f %.3f ", //足端速度
                    leg_drv->state[leg].v(0),
                    leg_drv->state[leg].v(1),
                    leg_drv->state[leg].v(2));
    }  
    fprintf(data_file, "\n");//结束，换行                 
}

/**
 * @description: 保存所有数据
 * @param {无}
 * @return {无}
 */
void leg_controller::data_all()
{
    // 在终端上直接输出传感器采集板的数据
    for(int i = 0; i < 6; i++){
        printf("%.3f  ", usb_data.sdata[i]);
    }printf("\n");


    //第1列，实时时间
    fprintf(data_file, "%.3f ", real_t);
    //第2列，当前控制模式
    fprintf(data_file, "%d ", mode_ctr->mode);
    // //第3列，状态检查错误代码
    // fprintf(data_file, "%d ", safty_ck->state_check_code);
    // //第4列，指令检查错误代码
    // fprintf(data_file, "%d ", safty_ck->cmd_check_code);
    // //第5列，控制器运行一次的真实时间
    //fprintf(data_file, "%.3f ", ctr_dt);
    // fprintf(data_file, "%.3f ", mode_ctr->power_on_time_pub);

    // printf("mode:%d ", mode_ctr->mode);
    // printf("mpc_time:%.3f\n", mode_ctr->locomotion_ctr->mpc_real_dt);
    // printf("%.3f \n",    dynamic_real_dt);
    //  printf("%.3f ",    mode_ctr->power_on_time_pub);
    //  printf("%.3f ",    dynamic->time_C);
    //  printf("%.3f \n",  dynamic->time_H);
    
    // //第6列，mpc运行一次的真实时间
    // fprintf(data_file, "%.3f ", mode_ctr->locomotion_ctr->mpc_real_dt);
    // //第7列，动力学运行一次的真实时间
    // fprintf(data_file, "%.3f ", dynamic_real_dt);

    //共4列，支撑标志
    for(int leg = 0; leg < 4; leg++)
    {
        fprintf(data_file, "%d ", mode_ctr->locomotion_ctr->trot_gait.stance_flag[leg]);
    }
    // //共4列，支撑状态
    // for(int leg = 0; leg < 4; leg++)
    // {
    //     fprintf(data_file, "%.3f ", mode_ctr->locomotion_ctr->trot_gait.stance_state[leg]);
    // }
    // //共4列，摆动状态
    // for(int leg = 0; leg < 4; leg++)
    // {
    //     fprintf(data_file, "%.3f ", mode_ctr->locomotion_ctr->trot_gait.swing_state[leg]);
    // }
    //共4列，预测概率
    for(int leg = 0; leg < 4; leg++)
    {
        fprintf(data_file, "%.3f ", contact->pp[leg]);
    }
    //共4列，足端z向接触力
    for(int leg = 0; leg < 4; leg++)
    {
        fprintf(data_file, "%.3f ", dynamic->Fc[leg](2));
    }
    //共4列，测量概率
    for(int leg = 0; leg < 4; leg++)
    {
        fprintf(data_file, "%.3f ", contact->pm[leg]);
    }       
    //共4列，融合概率
    for(int leg = 0; leg < 4; leg++)
    {
        fprintf(data_file, "%.3f ", contact->pc[leg]);
    }  
    //共4列，无动力学足端z向接触力
    for(int leg = 0; leg < 4; leg++)
    {
        fprintf(data_file, "%.3f ", dynamic->Fc0[leg](2));
    }
    // //共4列，伪足端z向接触力
    // for(int leg = 0; leg < 4; leg++)
    // {
    //     fprintf(data_file, "%.3f ", dynamic->Fc_fake[leg](2));
    // }

    //共4列，关节合外力
    for(int leg = 0; leg < 4; leg++)
    {
        fprintf(data_file, "%.3f ", dynamic->Tau[leg](2));
    }
    //共4列，关节驱动力
    for(int leg = 0; leg < 4; leg++)
    {
        fprintf(data_file, "%.3f ", leg_drv->state[leg].tau(2));
    }


    // //共10列，遥控器指令
    // fprintf(data_file, "%d %d %d %d %d %.3f %.3f %.3f %.3f %.3f ", 
    //                           rc_cmd.SWA, rc_cmd.SWB, rc_cmd.SWC,
    //                           rc_cmd.SWE, rc_cmd.SWG, rc_cmd.LRO,
    //                           rc_cmd.LCO, rc_cmd.RRO, rc_cmd.RCO,
    //                           rc_cmd.VB);
    
    //共3*9列，机身状态估计数据
    // fprintf(data_file, "%.3f %.3f %.3f ", //姿态欧拉角，世界坐标系下，机身相对于世界坐标系
    //                     body_data->rpy(0),
    //                     body_data->rpy(1),
    //                     body_data->rpy(2));
    // fprintf(data_file, "%.3f %.3f %.3f ", //机身速度，机身坐标系下，机身相对于世界坐标系
    //                     body_data->v_body(0),
    //                     body_data->v_body(1),
    //                     body_data->v_body(2));
    // fprintf(data_file, "%.3f %.3f %.3f ", //机身速度，世界坐标系下，机身相对于世界坐标系
    //                     body_data->v_world(0),
    //                     body_data->v_world(1),
    //                     body_data->v_world(2));
    // fprintf(data_file, "%.3f %.3f %.3f ", //机身相对于世界坐标系的位置在世界坐标系下的表示
    //                     body_data->position(0),
    //                     body_data->position(1),
    //                     body_data->position(2));
    // fprintf(data_file, "%.3f %.3f %.3f ", //机身角速度，机身坐标系下，机身相对于世界坐标系
    //                     body_data->omega_body(0),
    //                     body_data->omega_body(1),
    //                     body_data->omega_body(2));
    // fprintf(data_file, "%.3f %.3f %.3f ", //机身角速度，世界坐标系下，机身相对于世界坐标系
    //                     body_data->omega_world(0),
    //                     body_data->omega_world(1),
    //                     body_data->omega_world(2));
    // fprintf(data_file, "%.3f %.3f %.3f ", //机身角加速度，机身坐标系下，机身相对于世界坐标系
    //                     body_data->alpha_body(0),
    //                     body_data->alpha_body(1),
    //                     body_data->alpha_body(2));
    // fprintf(data_file, "%.3f %.3f %.3f ", //机身加速度，机身坐标系下，机身相对于世界坐标系
    //                     body_data->a_body(0),
    //                     body_data->a_body(1),
    //                     body_data->a_body(2));
    // fprintf(data_file, "%.3f %.3f %.3f ", //机身加速度，世界坐标系下，机身相对于世界坐标系
    //                     body_data->a_world(0),
    //                     body_data->a_world(1),
    //                     body_data->a_world(2));

    // //共4列，运动模式，步态规划，
    // for(int leg = 0; leg < 4; leg++)//各腿支撑状态
    //     fprintf(data_file, "%d ", mode_ctr->locomotion_ctr->trot_gait.stance_flag[leg]);

    // //共5列，运动模式，期望运动状态
    // fprintf(data_file, "%.3f %.3f %.3f %.3f %.3f ", 
    //                     mode_ctr->locomotion_ctr->vx_des,  //期望前后方向速度，前+
    //                     mode_ctr->locomotion_ctr->vy_des,  //期望左右方向速度，左+
    //                     mode_ctr->locomotion_ctr->vyaw_des,//期望偏航角速度，逆+
    //                     mode_ctr->locomotion_ctr->hf_des,  //期望抬腿高度
    //                     mode_ctr->locomotion_ctr->body_height);//机身目标高度

    //共12*5列，运动模式，轨迹参数和足端力参数
    // for(int leg = 0; leg < 4; leg++)//摆动轨迹起点
    //     for(int i = 0; i < 3; i++)
    //         fprintf(data_file, "%.3f ", mode_ctr->locomotion_ctr->pb_hf0[leg](i));
    // for(int leg = 0; leg < 4; leg++)//摆动轨迹终点
    //     for(int i = 0; i < 3; i++)
    //         fprintf(data_file, "%.3f ", mode_ctr->locomotion_ctr->pb_hf1[leg](i));
    // for(int leg = 0; leg < 4; leg++)//摆动轨迹当前位置点
    //     for(int i = 0; i < 3; i++)
    //         fprintf(data_file, "%.3f ", mode_ctr->locomotion_ctr->pb_hfc[leg](i));
    // for(int leg = 0; leg < 4; leg++)//mpc计算的各腿足端力
    //     for(int i = 0; i < 3; i++)
    //         fprintf(data_file, "%.3f ", mode_ctr->locomotion_ctr->f_ff[leg](i));
    // for(int leg = 0; leg < 4; leg++)//mpc计算的各腿足端力对应的关节力
    //     for(int i = 0; i < 3; i++)
    //         fprintf(data_file, "%.3f ", mode_ctr->locomotion_ctr->tau_ff[leg](i));
             
    //共12*5列，电机控制指令
    // for(int leg = 0; leg < 4; leg++)
    //     for(int i = 0; i < 3; i++)
    //         fprintf(data_file, "%.3f ", leg_drv->cmd[leg].q_des(i));
    // for(int leg = 0; leg < 4; leg++)
    //     for(int i = 0; i < 3; i++)
    //         fprintf(data_file, "%.3f ", leg_drv->cmd[leg].qd_des(i));
    // for(int leg = 0; leg < 4; leg++)
    //     for(int i = 0; i < 3; i++)
    //         fprintf(data_file, "%.3f ", leg_drv->cmd[leg].tau_ff(i));
    // for(int leg = 0; leg < 4; leg++)
    //     for(int i = 0; i < 3; i++)
    //         fprintf(data_file, "%.3f ", leg_drv->cmd[leg].kp(i,i));
    // for(int leg = 0; leg < 4; leg++)
    //     for(int i = 0; i < 3; i++)
    //         fprintf(data_file, "%.3f ", leg_drv->cmd[leg].kd(i,i));

    // 共12*6列，电机状态数据
    // for(int leg = 0; leg < 4; leg++)
    //     for(int i = 0; i < 3; i++)
    //         fprintf(data_file, "%.3f ", leg_drv->state[leg].q(i));
    // for(int leg = 0; leg < 4; leg++)
    //     for(int i = 0; i < 3; i++)
    //         fprintf(data_file, "%.3f ", leg_drv->state[leg].qd(i));
    // for(int leg = 0; leg < 4; leg++)
    //     for(int i = 0; i < 3; i++)
    //         fprintf(data_file, "%.3f ", leg_drv->state[leg].qdd(i));
    // for(int leg = 0; leg < 4; leg++)
    //     for(int i = 0; i < 3; i++)
    //         fprintf(data_file, "%.3f ", leg_drv->state[leg].tau(i));
    // for(int leg = 0; leg < 4; leg++)
    //     for(int i = 0; i < 3; i++)
    //         fprintf(data_file, "%.3f ", leg_drv->state[leg].p(i));
    // for(int leg = 0; leg < 4; leg++)
    //     for(int i = 0; i < 3; i++)
    //         fprintf(data_file, "%.3f ", leg_drv->state[leg].v(i));

    fprintf(data_file, "\n");//结束，换行
}



/**
 * @description: 状态输出函数，该函数由robot console的一个线程单独运行
 * @param {无}
 * @return {无}
 */
void leg_controller::run_print_state()
{
    // print_leg_cmd();
    // print_leg_state();
    // print_mode_state();
    // print_body_state();
    // print_foot_force();
}



/**
 * @description: 输出腿的指令
 * @param {无}
 * @return {无}
 */
void leg_controller::print_leg_cmd()
{
    for(int leg = 0; leg < 4; leg++)
    {
        printf("tx leg%d ",leg);
        // printf(" || %.4f, %.4f, %.4f", leg_drv->cmd[leg].q_des(0),  leg_drv->cmd[leg].q_des(1),  leg_drv->cmd[leg].q_des(2));
        // printf(" || %.4f, %.4f, %.4f", leg_drv->cmd[leg].qd_des(0), leg_drv->cmd[leg].qd_des(1), leg_drv->cmd[leg].qd_des(2));
        printf(" || %.4f, %.4f, %.4f", leg_drv->cmd[leg].tau_ff(0), leg_drv->cmd[leg].tau_ff(1), leg_drv->cmd[leg].tau_ff(2));
        // printf(" || %.4f, %.4f, %.4f", leg_drv->cmd[leg].kp(0,0),   leg_drv->cmd[leg].kp(1,1),   leg_drv->cmd[leg].kp(2,2));
        // printf(" || %.4f, %.4f, %.4f", leg_drv->cmd[leg].kd(0,0),   leg_drv->cmd[leg].kd(1,1),   leg_drv->cmd[leg].kd(2,2));
        printf("\n");
    }
}

/**
 * @description: 输出腿的状态
 * @param {无}
 * @return {无}
 */
void leg_controller::print_leg_state()
{
    for(int leg = 0; leg < 4; leg++)
    {
        printf("rx leg%d ",leg);
        // printf(" || %.4f, %.4f, %.4f", leg_drv->state[leg].q(0),   leg_drv->state[leg].q(1),   leg_drv->state[leg].q(2));
        // printf(" || %.4f, %.4f, %.4f", leg_drv->state[leg].qd(0),  leg_drv->state[leg].qd(1),  leg_drv->state[leg].qd(2));
        // printf(" || %.4f, %.4f, %.4f", leg_drv->state[leg].tau(0), leg_drv->state[leg].tau(1), leg_drv->state[leg].tau(2));
        printf(" || %.4f, %.4f, %.4f", leg_drv->state[leg].p(0),   leg_drv->state[leg].p(1),   leg_drv->state[leg].p(2));
        // printf(" || %.4f, %.4f, %.4f", leg_drv->state[leg].v(0),   leg_drv->state[leg].v(1),   leg_drv->state[leg].v(2));
        // printf(" || %.4f, %.4f, %.4f", leg_drv->state[leg].f(0),   leg_drv->state[leg].f(1),   leg_drv->state[leg].f(2));
        printf("\n");
    }
}

/**
 * @description: 输出当前的控制模式
 * @param {无}
 * @return {无}
 */
void leg_controller::print_mode_state()
{
    if(mode_ctr->mode == estop)      printf("mode: estop\n");
    if(mode_ctr->mode == power_off)  printf("mode: power_off\n");
    if(mode_ctr->mode == power_on)   printf("mode: power_on\n");
    if(mode_ctr->mode == fold)       printf("mode: fold\n");
    if(mode_ctr->mode == stand_up)   printf("mode: stand_up\n");
    if(mode_ctr->mode == stand_down) printf("mode: stand_down\n");
    if(mode_ctr->mode == balance)    printf("mode: balance\n");
    if(mode_ctr->mode == locomotion) printf("mode: locomotion\n");
}

/**
 * @description: 输出机身状态
 * @param {无}
 * @return {无}
 */
void leg_controller::print_body_state()
{
    printf("body state: ");
    // printf(" || %.4f, %.4f, %.4f", body_data->rpy(0), body_data->rpy(1), body_data->rpy(2));
    // printf(" || %.4f, %.4f, %.4f", body_data->omega_body(0), body_data->omega_body(1), body_data->omega_body(2));
    // printf(" || %.4f, %.4f, %.4f", body_data->omega_world(0), body_data->omega_world(1), body_data->omega_world(2));
    // printf(" || %.4f, %.4f, %.4f", body_data->a_body(0), body_data->a_body(1), body_data->a_body(2));
    // printf(" || %.4f, %.4f, %.4f", body_data->a_world(0), body_data->a_world(1), body_data->a_world(2));
    // printf(" || %.4f, %.4f, %.4f", body_data->position(0), body_data->position(1), body_data->position(2));
    // printf(" || %.4f, %.4f, %.4f", body_data->v_world(0), body_data->v_world(1), body_data->v_world(2));
    printf(" || %.4f, %.4f, %.4f", body_data->v_body(0), body_data->v_body(1), body_data->v_body(2));
    printf("\n");
}

/**
 * @description: 输出足端接触力
 * @param {无}
 * @return {无}
 */
void leg_controller::print_foot_force()
{
    
    // for(int leg = 0; leg < 4; leg++)
    // {
    //     printf("foot%d force: ",leg);
    //     printf("%.4f, %.4f, %.4f", dynamic->Fc[leg](0), dynamic->Fc[leg](1), dynamic->Fc[leg](2));
    //     printf("\n");
    // }

    printf("T: ");
    for(int i = 0; i < 9; i++) printf("%f, ",dynamic->T(i));
    printf("\n");
    printf("tau: %.4f, %.4f, %.4f\n", leg_drv->state[0].tau(0), leg_drv->state[0].tau(1), leg_drv->state[0].tau(2));
    printf("Tex: %.4f, %.4f, %.4f\n", dynamic->Tex[0](0), dynamic->Tex[0](1), dynamic->Tex[0](2));
    printf("Fc: %.4f, %.4f, %.4f\n", dynamic->Fc[0](0), dynamic->Fc[0](1), dynamic->Fc[0](2));
    Vec3<float> t = leg_drv->state[0].J.transpose() * dynamic->Fc[0];
    printf("Tex: %.4f, %.4f, %.4f\n", t(0), t(1), t(2));

}





