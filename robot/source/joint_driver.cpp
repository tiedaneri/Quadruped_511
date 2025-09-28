/*
 * @Author: sun
 * @Date: 2020-12-21 00:03:53
 * @LastEditTime: 2021-10-21 18:41:20
 * @LastEditors: Please set LastEditors
 * @Description: 机器人关节驱动源文件
 * @FilePath: /robot_software/robot/source/joint_driver.cpp
 */

/*
注，关节坐标系建立方式如下：
    所有的abad关节：机器人前进方向为旋转轴正方向，逆时针旋转为正
    所有的髋关节:   abad关节指向髋关节的方向为旋转轴正方向，逆时针旋转为正
    所有的膝关节:   abad关节指向髋关节的方向为旋转轴正方向，逆时针旋转为正

电机的旋转方向：
    输出轴伸出方向为旋转轴正方向，顺指针为正
*/
#include "joint_driver.h"

#include <math.h>  //包含"M_PI" 的定义
#include <string.h>//包含"memset" 的定义

//各腿各关节的实际o位置与设定位置的偏移量
const float abad_offset[4] = {0.025f, -0.025f, 0.f, 0.f};
const float hip_offset[4]  = {M_PI / 2.f, -M_PI / 2.f, -M_PI / 2.f, M_PI / 2.f};
const float knee_offset[4] = {4.35f, -4.35f, 4.35f, -4.35f};

//各腿各关节的实际位置与设定位置的缩放比例
const float abad_side_sign[4] = {-1.f, -1.f, 1.f, 1.f};
const float hip_side_sign[4]  = {-1.f, 1.f, -1.f, 1.f};
const float knee_side_sign[4] = {-.6429f, .6429f, -.6429f, .6429f};

//极限输出力矩
const float max_torque[3]      = {17.f, 17.f, 26.f}; 
const float wimp_torque[3]     = {6.f, 6.f, 6.f}; 
const float disabled_torque[3] = {0.f, 0.f, 0.f};

/**
 * @description: 构造函数，初始化参数
 * @param {无}
 * @return {无}
 */
joint_driver::joint_driver()
{
    //各变量清零
    memset(&cmd,      0, sizeof(cmd));
    memset(&state,    0, sizeof(state));
    memset(&torque,   0, sizeof(torque));
    memset(&tx_data1, 0, sizeof(tx_data1));
    memset(&tx_data2, 0, sizeof(tx_data2));
    memset(&rx_data1, 0, sizeof(rx_data1));
    memset(&rx_data2, 0, sizeof(rx_data2));
}

/**
 * @description: 初始化配置SPI1和SPI2
 * @param {无}
 * @return {0: 失败; 1: 成功}
 */
uint8_t joint_driver::init_spi()
{
    //初始化spi1
    spi1.port = "/dev/spidev2.0";
    if(!spi1.open_port()) return 0;
    if(!spi1.set_para())  return 0;
    //初始化spi2
    spi2.port = "/dev/spidev2.1";
    if(!spi2.open_port()) return 0;
    if(!spi2.set_para())  return 0;
    return 1;//成功
}

/**
 * @description: 亦或计算
 * @param {
 *           data: 需要计算的数据
 *           len : 数据的长度，单位：32 bits
 *         }
 * @return {计算结果}
 */
uint32_t joint_driver::xor_checksum(uint32_t *data, size_t len) 
{
    uint32_t t = 0;
    for (size_t i = 0; i < len; i++) t = t ^ data[i];
    return t;
}

/**
 * @description: 将关节指令数据转变成SPI的发送数据
 * @param {无}
 * @return {无}
 */
void joint_driver::command2spi()
{
    for (int i = 0; i < 2; i++) 
    {
        //将[0][1]腿关节指令数据进行缩放和偏移操作，然后存储到spi1的发送数据变量中
        tx_data1.q_des_abad[i]  = cmd.q_des_abad[i]  * abad_side_sign[i] + abad_offset[i];
        tx_data1.q_des_hip[i]   = cmd.q_des_hip[i]   * hip_side_sign[i]  + hip_offset[i];
        tx_data1.q_des_knee[i]  = cmd.q_des_knee[i]  / knee_side_sign[i] + knee_offset[i];
        tx_data1.qd_des_abad[i] = cmd.qd_des_abad[i] * abad_side_sign[i];
        tx_data1.qd_des_hip[i]  = cmd.qd_des_hip[i]  * hip_side_sign[i];
        tx_data1.qd_des_knee[i] = cmd.qd_des_knee[i] / knee_side_sign[i];
        tx_data1.kp_abad[i]     = cmd.kp_abad[i];
        tx_data1.kp_hip[i]      = cmd.kp_hip[i];
        tx_data1.kp_knee[i]     = cmd.kp_knee[i];
        tx_data1.kd_abad[i]     = cmd.kd_abad[i];
        tx_data1.kd_hip[i]      = cmd.kd_hip[i];
        tx_data1.kd_knee[i]     = cmd.kd_knee[i];
        tx_data1.tau_abad_ff[i] = cmd.tau_abad_ff[i] * abad_side_sign[i];
        tx_data1.tau_hip_ff[i]  = cmd.tau_hip_ff[i]  * hip_side_sign[i];
        tx_data1.tau_knee_ff[i] = cmd.tau_knee_ff[i] * knee_side_sign[i];
        tx_data1.flags[i]       = cmd.flags[i];

        //将[2][3]腿关节指令数据进行缩放和偏移操作，然后存储到spi2的发送数据变量中
        tx_data2.q_des_abad[i]  = cmd.q_des_abad[i + 2]  * abad_side_sign[i + 2] + abad_offset[i + 2];
        tx_data2.q_des_hip[i]   = cmd.q_des_hip[i + 2]   * hip_side_sign[i + 2]  + hip_offset[i + 2];
        tx_data2.q_des_knee[i]  = cmd.q_des_knee[i + 2]  / knee_side_sign[i + 2] + knee_offset[i + 2];
        tx_data2.qd_des_abad[i] = cmd.qd_des_abad[i + 2] * abad_side_sign[i + 2];
        tx_data2.qd_des_hip[i]  = cmd.qd_des_hip[i + 2]  * hip_side_sign[i + 2];
        tx_data2.qd_des_knee[i] = cmd.qd_des_knee[i + 2] / knee_side_sign[i + 2];
        tx_data2.kp_abad[i]     = cmd.kp_abad[i + 2];
        tx_data2.kp_hip[i]      = cmd.kp_hip[i + 2];
        tx_data2.kp_knee[i]     = cmd.kp_knee[i + 2];
        tx_data2.kd_abad[i]     = cmd.kd_abad[i + 2];
        tx_data2.kd_hip[i]      = cmd.kd_hip[i + 2];
        tx_data2.kd_knee[i]     = cmd.kd_knee[i + 2];
        tx_data2.tau_abad_ff[i] = cmd.tau_abad_ff[i + 2] * abad_side_sign[i + 2];
        tx_data2.tau_hip_ff[i]  = cmd.tau_hip_ff[i + 2]  * hip_side_sign[i + 2];
        tx_data2.tau_knee_ff[i] = cmd.tau_knee_ff[i + 2] * knee_side_sign[i + 2];
        tx_data2.flags[i]       = cmd.flags[i + 2];
    }
    //计算和保存spi1、spi2的亦或校验值
    tx_data1.checksum = xor_checksum((uint32_t* )(&tx_data1), 32);
    tx_data2.checksum = xor_checksum((uint32_t* )(&tx_data2), 32);
}

/**
 * @description: 将spi接收数据转变成关节状态数据
 * @param {无}
 * @return {0: 失败; 1: 成功}
 */
uint8_t joint_driver::spi2state()
{
    uint32_t calc_checksum;//定义变量存储本地计算的亦或校验值
    //对spi1收到的数据进行亦或校验
    calc_checksum = xor_checksum((uint32_t* )(&rx_data1), 14);
    if (calc_checksum != (uint32_t)rx_data1.checksum)//检验亦或校验值是否相等
    {
        printf("[ERROR] joint_driver: spi1 CHECKSUM: 0x%hx EXPECTED 0x%hx\n", calc_checksum, rx_data1.checksum);
        return 0;//失败
    }
    //对spi2收到的数据进行亦或校验
    calc_checksum = xor_checksum((uint32_t *)(&rx_data2), 14);
    if (calc_checksum != (uint32_t)rx_data2.checksum)//检验亦或校验值是否相等
    {
        printf("[ERROR] joint_driver: spi2 CHECKSUM: 0x%hx EXPECTED 0x%hx\n", calc_checksum, rx_data2.checksum);
        return 0;//失败
    }
    for (int i = 0; i < 2; i++) 
    {
        float filter = 1;
        //将spi1接收数据转变成腿[0][1]的关节状态数据
        state.q_abad[i]  = (rx_data1.q_abad[i] - abad_offset[i]) * abad_side_sign[i];
        state.q_hip[i]   = (rx_data1.q_hip[i]  - hip_offset[i])  * hip_side_sign[i];
        state.q_knee[i]  = (rx_data1.q_knee[i] - knee_offset[i]) * knee_side_sign[i];
        state.qd_abad[i] = rx_data1.qd_abad[i] * abad_side_sign[i] * filter + (1 - filter) * state.qd_abad[i];
        state.qd_hip[i]  = rx_data1.qd_hip[i]  * hip_side_sign[i]  * filter + (1 - filter) * state.qd_hip[i];
        state.qd_knee[i] = rx_data1.qd_knee[i] * knee_side_sign[i] * filter + (1 - filter) * state.qd_knee[i];
        state.flags[i]   = rx_data1.flags[i];
    
        //将spi2接收数据转变成腿[2][3]的关节状态数据
        state.q_abad[i + 2]  = (rx_data2.q_abad[i] - abad_offset[i + 2]) * abad_side_sign[i + 2];
        state.q_hip[i + 2]   = (rx_data2.q_hip[i]  - hip_offset[i + 2])  * hip_side_sign[i + 2];
        state.q_knee[i + 2]  = (rx_data2.q_knee[i] - knee_offset[i + 2]) * knee_side_sign[i + 2];
        state.qd_abad[i + 2] = rx_data2.qd_abad[i] * abad_side_sign[i + 2] * filter + (1 - filter) * state.qd_abad[i + 2];
        state.qd_hip[i + 2]  = rx_data2.qd_hip[i]  * hip_side_sign[i + 2]  * filter + (1 - filter) * state.qd_hip[i + 2];
        state.qd_knee[i + 2] = rx_data2.qd_knee[i] * knee_side_sign[i + 2] * filter + (1 - filter) * state.qd_knee[i + 2];
        state.flags[i + 2]   = rx_data2.flags[i];
    }
    return 1;//成功
}


static float old_qd_abad[4] = {0,0,0,0};
static float old_qd_hip[4]  = {0,0,0,0};
static float old_qd_knee[4] = {0,0,0,0};
/**
 * @description: 离散时间对速度微分，估计加速度
 * @param {无}
 * @return {无}
 */
void joint_driver::acc_estimate()
{
    float filter = 0.4;
    for(int leg = 0; leg < 4; leg++)
    {
        state.qdd_abad[leg] = ((state.qd_abad[leg] - old_qd_abad[leg]) / 0.002) * filter + (1 - filter) * state.qdd_abad[leg];
        state.qdd_hip[leg]  = ((state.qd_hip[leg]  - old_qd_hip[leg])  / 0.002) * filter + (1 - filter) * state.qdd_hip[leg];
        state.qdd_knee[leg] = ((state.qd_knee[leg] - old_qd_knee[leg]) / 0.002) * filter + (1 - filter) * state.qdd_knee[leg];

        old_qd_abad[leg] = state.qd_abad[leg];
        old_qd_hip[leg]  = state.qd_hip[leg];
        old_qd_knee[leg] = state.qd_knee[leg];
    }
}


/**
 * @description: 模拟电机的底层控制方法，根据当前发送的指令和返回的状态估计关节驱动力矩
 * @param {无}
 * @return {无}
 */
void joint_driver::torque_estimate()
{
    //定义指针变量存储驱动力矩的上限
    const float *torque_limits = disabled_torque;
    for(int i = 0; i < 4; i++)
    {
        //估计力矩，模拟电机底层的位置PD+前馈的控制方法
        torque.tau_abad[i] = cmd.kp_abad[i] * (cmd.q_des_abad[i]  - state.q_abad[i])  +
                             cmd.kd_abad[i] * (cmd.qd_des_abad[i] - state.qd_abad[i]) + 
                             cmd.tau_abad_ff[i];
        torque.tau_hip[i]  = cmd.kp_hip[i]  * (cmd.q_des_hip[i]   - state.q_hip[i])   +
                             cmd.kd_hip[i]  * (cmd.qd_des_hip[i]  - state.qd_hip[i])  +
                             cmd.tau_hip_ff[i];
        torque.tau_knee[i] = cmd.kp_knee[i] * (cmd.q_des_knee[i]  - state.q_knee[i])  +
                             cmd.kd_knee[i] * (cmd.qd_des_knee[i] - state.qd_knee[i]) +
                             cmd.tau_knee_ff[i];
        torque_limits = disabled_torque;//初始驱动力矩上限
        //根据当前指令设置驱动力矩上限
        if (cmd.flags[i] & 0b1)//使能标志的最低位为1时，表示电机使能
        {
            //使能标志的第二低位为1时，小力矩输出
            //使能标志的第二低位为0时，大力矩输出
            if (cmd.flags[i] & 0b10) torque_limits = wimp_torque;
            else torque_limits = max_torque;
        }
        //根据力矩上限修改估计的力矩值
        if (torque.tau_abad[i] >  torque_limits[0]) torque.tau_abad[i] =  torque_limits[0];
        if (torque.tau_abad[i] < -torque_limits[0]) torque.tau_abad[i] = -torque_limits[0];
        if (torque.tau_hip[i]  >  torque_limits[1]) torque.tau_hip[i]  =  torque_limits[1];
        if (torque.tau_hip[i]  < -torque_limits[1]) torque.tau_hip[i]  = -torque_limits[1];
        if (torque.tau_knee[i] >  torque_limits[2]) torque.tau_knee[i] =  torque_limits[2];  
        if (torque.tau_knee[i] < -torque_limits[2]) torque.tau_knee[i] = -torque_limits[2];

        //伪力矩
        torque.fake_tau_abad[i] = cmd.kp_abad[i] * (cmd.q_des_abad[i]  - state.q_abad[i] - 0.05 * state.qd_abad[i]);
        torque.fake_tau_hip[i]  = cmd.kp_hip[i]  * (cmd.q_des_hip[i]   - state.q_hip[i]  - 0.05 * state.qd_hip[i]);
        torque.fake_tau_knee[i] = cmd.kp_knee[i] * (cmd.q_des_knee[i]  - state.q_knee[i] - 0.05 * state.qd_knee[i]);
    }
}

/**
 * @description: 发送关节指令和更新关节状态
 * @param {无}
 * @return {0: 失败; 1: 成功}
 */
uint8_t joint_driver::update_data()
{
    command2spi();//将关节指令数据转换成spi发送数据
    //收发spi数据
    if(!spi1.send_receive((uint16_t* )(&tx_data1), (uint16_t* )(&rx_data1), (uint32_t)66, (uint32_t)30)) return 0;
    if(!spi2.send_receive((uint16_t* )(&tx_data2), (uint16_t* )(&rx_data2), (uint32_t)66, (uint32_t)30)) return 0;
    if(!spi2state()) return 0;//将spi接收数据转换成关节状态数据
    acc_estimate();//估计加速度
    torque_estimate();//估计驱动力矩
    return 1;//成功
}
