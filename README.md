# 简介

    此工程为具有WBC的从机器人实机代码。

# 指南

```bash
0. 在根目录CMakeLists.txt选择机器人的宏定义
1. 在PC端的"robot_software/build"文件夹中打开终端
2. 编译该工程。在终端中执行以下指令：
    {cmake -DCMAKE_BUILD_TYPE=Release ..} # 用于设置为Release模式进行编译
    {make}
3. 将程序下载到机器人中。在终端执行以下指令：
    {../scripts/loading.sh}
4. 通过SSH连接机器人。在终端执行以下指令(密码: 123456):
    {ssh user@10.0.0.34} # 1号机名称为robot，即：{ssh robot@10.0.0.34}(密码：123456).
5. 打开机器人中的程序文件夹. 执行以下指令：
    {cd robot_software/build}
6. 在机器人上运行程序. 执行以下指令：
    {./run_robot.sh ./[user program name]}
7. 结束程序指令按键：
    {Ctrl + C}
```


# TIP

    0. 软件需要在mini-cheetah的up-board环境中运行
    1. "third_party"   文件夹: 第三方的原文件和头文件
    2. "robot"               文件夹: 与机器人底层有关的代码
    3. "user"                 文件夹: main函数和上层控制器的代码，为程序的入口
    4. "user/test"       文件夹: 测试底层驱动的代码
    4. "scripts"            文件夹: 与下载程序和配置upboard有关的脚本文件
