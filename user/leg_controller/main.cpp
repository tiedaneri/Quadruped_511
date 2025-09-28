/*
 * @Author: sun
 * @Date: 2021-01-20 09:16:35
 * @LastEditTime: 2021-04-13 05:06:06
 * @LastEditors: Please set LastEditors
 * @Description: 主函数源文件，程序入口
 * @FilePath: /robot_software/user/leg_controller/main.cpp
 */

#include "robot_console.h"//控制台
#include "leg_controller.h"//控制器

#include <stdio.h>
#include <csignal>//包含信号处理功能函数

//机器人控制台对象
robot_console* main_console;

/**
 * @description: 信号处理函数，
 *               在程序运行的终端输入"ctr+C"可触发此函数，
 *               并终止程序
 * @param {int} signum
 * @return {无}
 */
void signal_handler(int signum)
{
    printf("[signal_handler] capture signal: %d\n", signum);
    main_console->stop_cmd = 1;//终止程序指令
}

/**
 * @description: 主函数，程序入口
 * @param {无}
 * @return {*}
 */
int main()
{
    leg_controller leg_ctr;//创建leg_controller控制器对象
    main_console = new robot_console(&leg_ctr);//创建robot_console对象
    signal(SIGINT, signal_handler);//注册信号和信号处理函数
    if(!main_console->init_sum())//控制台执行初始化操作
    {
        printf("[ERROE] main: main_console.init_sum()\n");
        return 1;//初始化失败，退出
    }
    //初始化完成
    main_console->run_tasks();//创建并运行所有任务
    //控制台运行后进入死循环，直到收到程序终止指令

    //控制台程序运行结束
    delete main_console;//删除控制台对象，释放内存

    printf("[main] program ended\n");
    return 1;//程序终止
}
