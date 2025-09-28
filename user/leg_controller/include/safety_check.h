/*
 * @Author: sun
 * @Date: 2021-03-24 06:26:56
 * @LastEditTime: 2021-10-01 06:27:32
 * @LastEditors: Please set LastEditors
 * @Description: safety check头文件
 *               检查机器人状态数据和指令数据，判断是否安全
 * @FilePath: /robot_software/user/leg_controller/include/safety_check.h
 */
/*
指南:
      0. 控制器程序开头执行状态检查：run_state_check
      1. 控制器程序结尾执行指令检查：run_cmd_check
*/
#ifndef SAFETY_CHECK_H
#define SAFETY_CHECK_H

#include "leg_driver.h"
#include "up_at9s.h"
#include "mode_controller.h"
#include "body_state_estimator.h"  //机身状态估计

class safety_check
{
  public:
    safety_check(leg_driver* _leg_drv, at9s_cmd* _rc_cmd, mode_controller* _mode_ctr, body_state_estimator_data* _body_data);

    void run_state_check(void);
    void run_cmd_check(void);

    int state_check_code;//状态检查错误代码
    int cmd_check_code;//指令检查错误代码

  private:
    void torque_state_check(void);     //1
    void orientation_state_check(void);//2
    void fold_state_check(void);       //3
    void stand_state_check(void);      //4
    void balance_state_check(void);    //5
    void locomotion_state_check(void); //6

    void torque_cmd_check(void);    //1   
    void jpos_cmd_check(void);      //2
    void fold_cmd_check(void);      //3
    void stand_cmd_check(void);     //4
    void balance_cmd_check(void);   //5
    void locomotion_cmd_check(void);//6
    
    //腿驱动对象，驱动腿并存储指令和状态数据
    leg_driver* leg_drv;
    //遥控器指令变量，存储遥控器指令
    at9s_cmd* rc_cmd;
    //模式控制对象，存储当前模式数据
    mode_controller* mode_ctr;
    //变量，存储估计的身体状态结果
    body_state_estimator_data* body_data;
};

#endif //#ifndef SAFETY_CHECK_H
