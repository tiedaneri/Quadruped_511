/*
 * @Author: sun
 * @Date: 2021-01-03 22:50:46
 * @LastEditTime: 2025-09-18 11:21:38
 * @LastEditors: Please set LastEditors
 * @Description: 机器人控制台头文件
 *               初始化硬件配置，管理数据，管理和启动控制任务
 * @FilePath: /robot_software/robot/source/robot_console.cpp
 */

#include "robot_console.h"
#include "robot.h"//包含控制周期：dt

#include <string.h>//包含"memset" 定义

#include <sys/mman.h>//prefaultStack() 中的参数

#define MAX_STACK_SIZE 16384  // 16KB  of stack
#define TASK_PRIORITY 49      // linux priority, this is not the nice value

/**
 * @description: 构造函数，初始化参数
 * @param {无}
 * @return {无}
 */
robot_console::robot_console(robot_controller* rt_ctr)
{
    robot_ctr = rt_ctr;//连接控制器
    stop_cmd = 0;//设置停止运行标志，程序处于运行状态
}

/**
 * @description: 析构函数
 * @param {无}
 * @return {无}
 */
robot_console::~robot_console()
{
    printf("[robot_console] robot_console ended\n");
}

/**
 * @description: 执行所有的初始化操作
 * @param {无}
 * @return {0: 失败; 1: 成功}
 */
uint8_t robot_console::init_sum()
{
    if(!prefaultStack()) return 0;
    if(!setupScheduler()) return 0;
    
    if(!init_hardware()) return 0;//初始化硬件配置
    init_controller();//初始化控制软件

    return 1;//成功
}

/*!
 * Writes to a 16 KB buffer on the stack. If we are using 4K pages for our
 * stack, this will make sure that we won't have a page fault when the stack
 * grows.  Also mlock's all pages associated with the current process, which
 * prevents the cheetah software from being swapped out.  If we do run out of
 * memory, the robot program will be killed by the OOM process killer (and
 * leaves a log) instead of just becoming unresponsive.
 */
/**
 * @description: 不知道这是什么东西
 * @param {无}
 * @return {0: 失败; 1: 成功}
 */
uint8_t robot_console::prefaultStack() 
{
    printf("[Init] Prefault stack...\n");
    volatile char stack[MAX_STACK_SIZE];
    memset(const_cast<char*>(stack), 0, MAX_STACK_SIZE);
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) 
    {
        printf("[ERROR]: robot_console: init Prefault stack!!\n");
        return 0;//失败
    }
    return 1;//成功
}

/*!
 * Configures the scheduler for real time priority
 */
/**
 * @description: 不知道这是什么东西
 * @param {无}
 * @return {0: 失败; 1: 成功}
 */
uint8_t robot_console::setupScheduler() 
{
    printf("[Init] Setup RT Scheduler...\n");
    struct sched_param params;
    params.sched_priority = TASK_PRIORITY;
    if (sched_setscheduler(0, SCHED_FIFO, &params) == -1) 
    {
        printf("[ERROR]: robot_console: setupScheduler!!\n");
        return 0;//失败
    }
    return 1;//成功
}

/**
 * @description: 初始化硬件
 * @param {无}
 * @return {0: 失败; 1: 成功}
 */
uint8_t robot_console::init_hardware()
{
    if(!usb_dev.init_usb()){//传感器采集板(USB设备)初始化
        printf("[ERROR]: robot_console: init usb device!!\n");
        return 0;
    }
    printf("[robot_console] init usb device success!!\n");
    if(!at9s.init_serial())//遥控器初始化
    {
        printf("[ERROR]: robot_console: init serial!!\n");
        return 0;//失败
    }
    printf("[robot_console] init serial port success!!\n");
    if(!joint_drv.init_spi())//关节驱动初始化
    {
        printf("[ERROR]: robot_console: init spi\n");
        return 0;//失败
    }
    printf("[robot_console] init spi success!!\n");
    if(!imu.tryInit(0, 460800))//imu初始化
    {
        printf("[ERROR]: robot_console: init imu\n");
        return 0;//失败
    }
    return 1;//成功
}

/**
 * @description: 初始化控制软件
 * @param {无}
 * @return {无}
 */
void robot_console::init_controller()
{
    body_se.leg_state = leg_drv.state;//连接腿状态数据到身体状态估计
    body_se.init();//初始化身体状态估计

    robot_ctr->body_data = &body_se.body_data;//连接机身状态估计数据到控制器
    robot_ctr->leg_drv = &leg_drv;//连接腿驱动到控制器
    robot_ctr->init();//初始化控制器
}

/**
 * @description: 创建和启动任务
 * @param {无}
 * @return {无}
 */
void robot_console::run_tasks()
{
    //创建传感器采集板(USB设备)任务
    PeriodicMemberFunction<robot_console> usb_device_task(&taskmanager, .004, "usb_device_task", &robot_console::run_usb_device, this);    
    //创建遥控器任务
    PeriodicMemberFunction<robot_console> remote_task(&taskmanager, .005, "remote_task", &robot_console::run_remote, this);
    //创建imu任务
    PeriodicMemberFunction<robot_console> imu_task(&taskmanager, .002, "imu_task", &robot_console::run_imu, this);
    //创建关节驱动任务
    PeriodicMemberFunction<robot_console> joint_drive_task(&taskmanager, .002, "joint_drive_task", &robot_console::run_joint_drive, this);
    //创建控制器任务
    PeriodicMemberFunction<robot_console> robot_controller_task(&taskmanager, robot::dt, "robot_controller_task", &robot_console::run_robot_controller, this);
    
    usb_device_task.start();      //启动传感器采集板(USB设备)任务
    remote_task.start();          //启动遥控器任务
    imu_task.start();             //启动imu任务
    joint_drive_task.start();     //启动关节驱动任务
    usleep(1000000);//延时1s，等待更新硬件数据
    robot_controller_task.start();//启动控制器任务

    
    while(!stop_cmd)//检测是否收到程序终止指令
    {
        usleep(1000000);//1s
    }
    taskmanager.stopAll();//停止所有任务
    printf("[robot_console] all task closed\n");
}

/**
 * @description: 传感器采集板(USB设备)任务函数
 * @param {无}
 * @return {无}
 */
void robot_console::run_usb_device()
{
    // 读取USB设备数据
    if(usb_dev.read_and_parse_data()) {
        usb_data_m.lock();           //USB数据线程锁，上锁
        // 将USB设备数据复制到robot_console
        memcpy(&usb_data, &usb_dev.data, sizeof(usb_device_data));
        usb_data_m.unlock();        //USB数据线程锁，解锁
    }
}

/**
 * @description: 遥控器任务函数
 * @param {无}
 * @return {无}
 */
void robot_console::run_remote()
{
    at9s.update_cmd();//接收并解析指令
    
    rcmd_m.lock();//遥控器指令数据线程锁，上锁
    //将遥控器数据复制到robot_console
    memcpy(&rc_cmd, &at9s.cmd, sizeof(at9s_cmd));
    rcmd_m.unlock();//遥控器指令数据线程锁，解锁
}

/**
 * @description: imu任务函数
 * @param {无}
 * @return {无}
 */
void robot_console::run_imu()
{
    imu.run();//更新数据

    imu_m.lock();//imu数据线程锁，上锁
    //将imu数据复制到robot_console
    for(int i = 0; i < 4; i++) quat[i] = imu.quat(i);
    for(int i = 0; i < 3; i++)
    {
        gyro[i] = imu.gyro(i);
        acc[i]  = imu.acc(i);
    }
    imu_m.unlock();//imu数据线程锁，解锁
}

/**
 * @description: 关节驱动任务函数
 * @param {无}
 * @return {无}
 */
void robot_console::run_joint_drive()
{
    jcmd_m.lock();//关节指令数据上锁
    //将关节指令数据从robot_console复制到关节驱动
    memcpy(&joint_drv.cmd, &joint_cmd, sizeof(joint_command_data));
    jcmd_m.unlock();//关节指令数据解锁

    //关节驱动更新状态数据
    joint_drv.update_data();

    jdata_m.lock();//关节状态数据上锁
    //将关节状态数据复制到robot_console
    memcpy(&joint_state, &joint_drv.state, sizeof(joint_state_data));
    memcpy(&joint_torque, &joint_drv.torque, sizeof(joint_torque_estimate));
    jdata_m.unlock();//关节状态数据解锁

}

/**
 * @description: 控制器任务函数
 * @param {无}
 * @return {无}
 */
void robot_console::run_robot_controller()
{
    usb_data_m.lock();           //USB数据线程锁，上锁
    //将传感器采集板数据从robot_console复制到控制器
    memcpy(&robot_ctr->usb_data, &usb_data, sizeof(usb_data));
    usb_data_m.unlock();         //USB数据线程锁，解锁

    rcmd_m.lock();//遥控器数据上锁
    //将遥控器数据从robot_console复制到控制器
    memcpy(&robot_ctr->rc_cmd, &rc_cmd, sizeof(at9s_cmd));
    rcmd_m.unlock();//遥控器数据解锁

    // printf("[leg_controller]: SWE->%d\n", robot_ctr->rc_cmd.SWE); //打印leg_controller的rc_cmd的SWE
    // printf("[leg_controller]: SWG->%d\n", robot_ctr->rc_cmd.SWG);

    jdata_m.lock();//关节驱动数据上锁
    //将关节驱动数据从robot_console复制到腿驱动
    memcpy(&leg_drv.joint_state, &joint_state, sizeof(joint_state_data));
    memcpy(&leg_drv.joint_torque, &joint_torque, sizeof(joint_torque_estimate));
    jdata_m.unlock();//关节驱动数据解锁

    imu_m.lock();//imu数据线程锁，上锁
    //将imu数据从robot_console复制到身体状态估计对象
    for(int i = 0; i < 4; i++) body_se.quat[i] = quat[i];
    for(int i = 0; i < 3; i++)
    {
        body_se.gyro[i] = gyro[i];
        body_se.acc[i] = acc[i];
    }
    imu_m.unlock();//imu数据线程锁，解锁

    leg_drv.update_state();//腿驱动更新状态数据
    leg_drv.zero_cmd();    //腿驱动清空指令数据

    body_se.run();//执行身体状态估计
    
    robot_ctr->run_controller();//运行控制器

    leg_drv.update_cmd();//腿驱动更新腿指令数据到关节指令

    jcmd_m.lock();//关节指令数据上锁
    //将关节指令数据从腿驱动更新到robot_console
    memcpy(&joint_cmd, &leg_drv.joint_cmd, sizeof(joint_command_data));
    jcmd_m.unlock();//关节指令数据上锁
}

