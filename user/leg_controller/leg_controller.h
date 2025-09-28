/*
 * @Author: sun
 * @Date: 2021-01-20 09:16:35
 * @LastEditTime: 2024-02-25 23:07:44
 * @LastEditors: Please set LastEditors
 * @Description: 腿控制器头文件
 * @FilePath: /robot_software/user/leg_controller/leg_controller.h
 */
/*
指南:
      0. 创建对象时自动运行构造函数: leg_controller();
      1. 运行初始化函数: init();
      2. 循环运行控制器主程序: run_controller();
      3. 调试时可运行状态输出函数： run_print_state();
*/

#ifndef LEG_CONTROLLER_H
#define LEG_CONTROLLER_H

#include "robot_controller.h"

#include "mode_controller.h"//模式控制
#include "safety_check.h"//安全检查
#include "robot_model.h"
#include "fb_dynamic.h"
#include "contact_detection.h"
//包含Vec3<float>等与Eigen有关类型的定义
#include "cppTypes.h"

class leg_controller: public robot_controller
{
  public:
    leg_controller(void);
    ~leg_controller();

    void init(void);
    void run_controller(void);
    void run_print_state(void);
    void run_data_write(void);

  private:
    mode_controller* mode_ctr;//模式控制对象
    safety_check* safty_ck;//安全检查对象
    robot_model model;//机器人运动树模型
    fb_dynamic* dynamic;//浮动基系统动力学计算工具
    contact_detection* contact;//接触状态估计对象

    double real_t;//控制器运行计时
    double ctr_dt;//控制器运行一次实际时间
    double dynamic_real_dt;//记录动力学运行一次的真实时间
    
    FILE* data_file;//保存数据的文件

    void write_robot_data(void);

    void print_leg_cmd(void);
    void print_leg_state(void);
    void print_mode_state(void);
    void print_body_state(void);
    void print_foot_force(void);

    void data_all(void);
    void data_test1(void);
    void data_test2(void);
    void data_test3(void);
};

#endif
